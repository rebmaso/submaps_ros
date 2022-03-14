#include <SupereightInterface.hpp>

bool SupereightInterface::addDepthImage(const okvis::Time &stamp,
                                        const cv::Mat &depthFrame) {
  // Create an OKVIS Image measurement. (contains an image and a depthImage
  // member)
  CameraMeasurement depthMeasurement;
  // depthMeasurement.image = image;
  depthMeasurement.measurement.depthImage = depthFrame;
  depthMeasurement.measurement.deliversKeypoints = false;
  depthMeasurement.timeStamp = stamp;

  // Push data to the Queue.
  const size_t depthQueueSize =
      1000; ///< Arbitrary number. ToDo -> fine-tune once integration time has
            ///< been taken into account.
  if (blocking_) {
    const bool result =
        depthMeasurements_.PushBlockingIfFull(depthMeasurement, depthQueueSize);
    cvNewSensorMeasurements_.notify_one();
    return result;
  } else {
    // Push measurement and pop the oldest entry.
    const bool result = depthMeasurements_.PushNonBlockingDroppingIfFull(
        depthMeasurement, depthQueueSize);
    cvNewSensorMeasurements_.notify_one();

    if (result)
      LOG(WARNING) << "Oldest Depth measurement dropped";

    return true;
  }
  return false;
}

bool SupereightInterface::addImuMeasurement(const okvis::Time &stamp,
                                            const Eigen::Vector3d &alpha,
                                            const Eigen::Vector3d &omega) {
  // Create an okvis IMU measurement
  ImuMeasurement imuMeasurement;
  imuMeasurement.measurement.accelerometers = alpha;
  imuMeasurement.measurement.gyroscopes = omega;
  imuMeasurement.timeStamp = stamp;

  const size_t imuQueueSize = 50000; ///< ToDo- > fine-tune once integration
                                     ///< time has been taken into account.
  if (blocking_) {
    const bool result =
        imuMeasurements_.PushBlockingIfFull(imuMeasurement, imuQueueSize);
    // cvNewSensorMeasurements_.notify_one();

    return result;
  } else {
    // Push measurement and pop the oldest entry.
    if (imuMeasurements_.PushNonBlockingDroppingIfFull(imuMeasurement,
                                                       imuQueueSize)) {
      // Oldest measurement dropped.
      LOG(WARNING) << "Oldest imu measurement dropped";
      // cvNewSensorMeasurements_.notify_one();
      return true;
    }
  }
  return false;
}

bool SupereightInterface::dataReadyForProcessing() {
  // Get the timestamp of the oldest Depth frame
  CameraMeasurement oldestDepthMeasurement;
  if (!depthMeasurements_.getCopyOfFront(&oldestDepthMeasurement))
    return false;

  OkvisUpdate newestState;
  if (!stateUpdates_.getCopyOfBack(&newestState))
    return false;

  ImuMeasurement newestImuMeasurement;
  if (!imuMeasurements_.getCopyOfBack(&newestImuMeasurement))
    return false;

  return (oldestDepthMeasurement.timeStamp <= newestState.timestamp) &&
         (oldestDepthMeasurement.timeStamp <= newestImuMeasurement.timeStamp);
}

DepthFrame SupereightInterface::depthMat2Image(const cv::Mat &inputDepth) {
  
  // Scale the input depth based on TUM convension
  // needed this bc dataset is 1 mt --> 5000. With the simulated depth cam we should already have 1mt -> 1
  cv::Mat depthScaled;
  // inputDepth.convertTo(depthScaled, CV_32FC1, 1.f / 5000.f);
  inputDepth.convertTo(depthScaled, CV_32FC1);

  // Initialise and copy
  DepthFrame output(depthScaled.size().width, depthScaled.size().height, 0.f);

  // cv::MAT and DepthFrame keep data stored in row major format.
  memcpy(output.data(), depthScaled.data,
         depthScaled.size().width * depthScaled.size().height * sizeof(float));


  // cv::Mat test;
  // inputDepth.convertTo(test, CV_32FC1, 1.f / 5000.f);
  // memcpy(test.data, output.data(),
  //        test.size().width * test.size().height * sizeof(float));
  // cv::imshow("test", test);
  // cv::imshow("depthScaled", depthScaled);

  return output;
}

cv::Mat SupereightInterface::depthImage2Mat(const DepthFrame &depthFrame) {
  cv::Mat outputMat(depthFrame.height(), depthFrame.width(), CV_16UC1);

  // Iterate throught all the pixels.
  outputMat.forEach<uint16_t>([&](uint16_t &p, const int *position) -> void {
    const uint idx = depthFrame.width() * position[0] + position[1];

    // Scale measurement

    p = static_cast<uint16_t>(depthFrame[idx] * 5000.0f);
  });
  return outputMat;
}

bool SupereightInterface::predict(const okvis::Time &finalTimestamp,
                                  Transformation &T_WC0, Transformation &T_WC,
                                  uint64_t &keyframeId,
                                  KeyFrameDataVec &keyFrameDataVec) {
  // Locate the state timestamp closest to the finalTimestamp. Todo-> Switch the
  // backend of the threadsafe Queue to a deque ToDo -> Queue is sorted, use
  // binary search instead of linear Search

  // Get the okvis update closest to the finalTimestamp -> then we predict with imu integration
  OkvisUpdate initialStateData;
  OkvisUpdate initialStateTemp;
  while (stateUpdates_.getCopyOfFrontBlocking(&initialStateTemp)) {
    // Check the timestamp
    if (initialStateTemp.timestamp > finalTimestamp)
      break;
    initialStateData = initialStateTemp;

    // Pop from the Queue
    stateUpdates_.PopNonBlocking(&initialStateTemp);
  }

  // Check that the initialStateData is initialised. Todo-> Clarify why we need
  // this
  if (initialStateData.keyframeStates.empty())
    return false;

  // Retrieve the Pose of the latest inserted Keyframe (active KF tied to initial state). ToDo -> This is only
  // correct when the vector has been created withopur altering the KF order
  const auto &T_WS0 = initialStateData.keyframeStates.rbegin()->T_WS;
  T_WC0 = T_WS0 * T_SC_;

  // Id of the latest inserted Kf.
  keyframeId = initialStateData.keyframeStates.rbegin()->id.value();

  // KF poses. 
  keyFrameDataVec = KeyFrameDataVec(initialStateData.keyframeStates.size());
  for (size_t i = 0; i < initialStateData.keyframeStates.size(); i++) {
    const auto &state = initialStateData.keyframeStates[i];
    keyFrameDataVec.at(i) = KeyframeData(state.id.value(), state.T_WS * T_SC_);
  }

  // We can use the built in OKVIS prediction. Todo -> try to avoid the
  // duplicate queues and duplicate  code and check if we can directly Query the
  // propagated pose from OKVIS
  okvis::ImuMeasurementDeque imuDeque;
  while (true) {
    // Get a copy of the oldest element in the Queue.
    ImuMeasurement imuMeasurement;
    if (!imuMeasurements_.getCopyOfFront(&imuMeasurement))
      return false;

    // Check timestamp. If greater than final timestamp, continue;
    const double imuDt =
        5 * 1e-3; // ToDo -> set this based on the actual IMU rate
    if (imuMeasurement.timeStamp > finalTimestamp + okvis::Duration(imuDt)) {
      break;
    }

    // Push to the Deque and Pop from Measurement Queue.
    imuDeque.push_back(imuMeasurement);

    // Pop from Queue
    // Todo -> this should only work if no other thread is trying to pop at the
    // same time. Add a lock to prevent this from happening.
    imuMeasurements_.PopNonBlocking(&imuMeasurement);
  }

  // Perform the prediction
  okvis::kinematics::Transformation T_WS(initialStateData.latestState.T_WS);
  okvis::SpeedAndBias speedAndBiases(
      (okvis::SpeedAndBias() << initialStateData.latestState.v_W,
       initialStateData.latestState.b_g, initialStateData.latestState.b_a)
          .finished());

  // Todo -> Load the parameters from a config file. Set to a large value to
  // supress warnings.
  okvis::ImuParameters imuParams;
  imuParams.a_max = 1e+4;
  imuParams.g_max = 1e+4;

  const int integrationNo = okvis::ceres::ImuError::propagation(
      imuDeque, imuParams, T_WS, speedAndBiases, initialStateData.timestamp,
      finalTimestamp);
  if (integrationNo == -1)
    return false;

  // Predicted Pose
  T_WC = T_WS * T_SC_;

  return true;
}

void SupereightInterface::processSupereightFrames() {

  // Wake Up on arrival of new measurements
  unsigned frame = 0;

  while (true) {

    // Wait on Condition variable signaling
    std::unique_lock<std::mutex> lk(s8Mutex_);
    cvNewSupereightData_.wait(lk, [&] { return supereightFrames_.Size(); });

    // Get the supereight depth frame --> need to integrate it into a submap
    SupereightFrame supereightFrame;
    if (!supereightFrames_.PopNonBlocking(&supereightFrame))
      continue;

    // Prevent other thread from editing the submaps vector 
    std::unique_lock<std::mutex> submapLock(subMapMutex_);

    // Proceed with supereight integration
    const auto start_time = std::chrono::high_resolution_clock::now();

    // Update pose lookup --> each time a new seframe arrives, it contains updated info on all kf poses.
    //  we update here all poses
    for (auto &keyframeData : supereightFrame.keyFrameDataVec) {
    
      const auto id = keyframeData.id;
      const auto T_WM = keyframeData.T_WM;
      
      Eigen::Vector3i pos_floor; //  floor of kf position
      for (int i = 0 ; i < 3 ; i++)
      {
        pos_floor(i) = (int)(floor(T_WM.r()(i)));
      }

      // update hash table: push map id in the boxes inside radius (avoid adding same id 2 times in a given box)
      
      // simple version: real map is a 5x5x5 box in front of the camera. 
      // lets assign a 10x10x10 set of 1x1x1 boxes around the kf so it's simple
      // will obviously exceed real submap boundaries but theres no problem w that.
      // instead, it increases efficiency for huge maps
      // side note: -0.5 and 0.5 both map in the 0 box coordinate. 
      // this is not cool, but also not a problem (its not a real bug, planning works fine)

      const int side = 5; // TODO get this from the map config?
      for (int x = -side; x < side; x++)
      {
        for (int y = -side; y < side; y++)
        {
          for (int z = -side; z < side; z++)
          {
            // this statement is executed 1000 times... maybe decrease box radius
            const Eigen::Vector3i offset(x,y,z);
            hashTable_[pos_floor + offset].insert(id); // uniquely add id
          }
        }
      }
      
      // Check If Id exists.
      if (submapPoseLookup_.count(id)) {
        // Update.
        submapPoseLookup_[id] = T_WM;
      } else {
        // Insert
        submapPoseLookup_.insert(std::make_pair(id, T_WM));
      }
    }

    // Chech whether we need to create a new submap. --> integrate in new or existing map?
    // this is the latest "active" keyframe id
    static uint64_t prevKeyframeId = supereightFrame.keyframeId;

    // ---------------------------- CREATE NEW MAP ---------------------------

    if (supereightFrame.keyframeId != prevKeyframeId || submaps_.empty()) { 

      // Quick hack for Tomaso, save the finished map using its current pose.
      if (!submaps_.empty()) {
        const std::string meshFilename =
            meshesPath_ + "/" + std::to_string(prevKeyframeId) + ".ply";

        // Retrieve the respective KF pose from the lookup we just updated by reading seframes
        const auto T_WF = submapPoseLookup_[prevKeyframeId];

        // Compute the tranformation needed in Supereight 2. We can use
        // submaps_.back()->getTWM(). However this returns  a zero matrix in s8
        // e9fa2e01e3124ae76896393450f21180fae76cf4. Let's instead get the
        // transformation from the map config.
        const Eigen::Matrix4d T_MW = mapConfig_.T_MW.cast<double>();
        Eigen::Matrix4d T_WM = Eigen::Matrix4d::Identity();

        // Compute the inverse manually....
        T_WM.topLeftCorner<3, 3>() = T_MW.topLeftCorner<3, 3>().transpose();
        T_WM.topRightCorner<3, 1>() = -T_MW.topLeftCorner<3, 3>().transpose() *
                                      T_MW.topRightCorner<3, 1>();
        
        // this should be the mesh tf wrt world (bc Twm should be T(sensor)(mesh))
        // Eigen::Matrix4d T_WM_mesh = T_WF.T() * T_WM;
        
        // --------------- !!!!!!!!!!!!!!!!!!!!!! ------------------
        // since we only need to visualize them w markers. lets save them with identity pose (but w the right scale), and add the kf pose later
        Eigen::Matrix4d T_WM_mesh = T_WM;

        // Apply the scale. Important -> this interface will change in the next
        // versions of supereight 2.0
        T_WM_mesh.topLeftCorner<3, 3>() =
            submaps_.back()->getRes() * T_WM_mesh.topLeftCorner<3, 3>();

        // SAVE THE MESH (its name is the kf id) ADDING AN OFFSET OF TWM (THE KF POSE)
        // std::cout << "(seinterface) saving mesh as ply... \n";
        submaps_.back()->saveMesh(meshFilename, T_WM_mesh.cast<float>()); 
                
        // call submap visualizer & plan() (outside this class) each time we save a new map
        publishSubmaps(); 
        replan();
      }

      // Create a new submap and reset frame counter
      submaps_.emplace_back(
          new se::OccupancyMap<se::Res::Multi>(mapConfig_, dataConfig_));
      frame = 0;

      // Add the (keyframe Id, iterator) pair in the submapLookup_
      submapLookup_.insert(std::make_pair(supereightFrame.keyframeId,
                                          std::prev(submaps_.end())));
    }

    // -------------------------------------------------------------------

    // INTEGRATE IN CURRENT ("active") MAP

    // Prepare for next iteration
    prevKeyframeId = supereightFrame.keyframeId;

    // Retrieve the active submap.
    auto &activeMap = submaps_.back();

    // Integrate depth frame
    se::MapIntegrator integrator(
        *activeMap); //< ToDo -> Check how fast this constructor is
    integrator.integrateDepth(sensor_, supereightFrame.depthFrame,
                              supereightFrame.T_WC.T().cast<float>(), frame);
    frame++;

    const auto current_time = std::chrono::high_resolution_clock::now();

    // // Some couts, remove when done debugging.
    // std::cout << "Processing delay\t"
    //           << (std::chrono::duration_cast<std::chrono::milliseconds>(
    //                   current_time - start_time))
    //                  .count()
    //           << "\t Integrated frames: " << frame
    //           << "\tS8 Fr: " << supereightFrames_.Size()
    //           << "\t submaps: " << submaps_.size()
    //           << "\t Okvis Upd: " << stateUpdates_.Size()
    //           << "\tDepth Fr: " << depthMeasurements_.Size() << std::endl;

  }
}

void SupereightInterface::pushSuperEightData() {
  // Wake up on the arrival of new measurements.
  while (true) {
    std::unique_lock<std::mutex> lk(cvMutex_);
    cvNewSensorMeasurements_.wait(
        lk, [&] { return this->dataReadyForProcessing(); });

    // Get the depth Image and convert to supereight format.
    CameraMeasurement depthMeasurement;
    if (!depthMeasurements_.PopNonBlocking(&depthMeasurement))
      continue;

    // Compute the respective pose using the okvis updates and the IMU
    // measurements.
    Transformation T_WC0, T_WC;
    uint64_t lastKeyframeId;
    KeyFrameDataVec keyFrameDataVec;
    if (!predict(depthMeasurement.timeStamp, T_WC0, T_WC, lastKeyframeId,
                 keyFrameDataVec))
      continue;

    // Construct Supereight Frame and push to the corresponding Queue
    const SupereightFrame supereightFrame(
        T_WC0.inverse() * T_WC,
        depthMat2Image(depthMeasurement.measurement.depthImage), lastKeyframeId,
        keyFrameDataVec);

    // Push to the Supereight Queue.
    const size_t supereightQueueSize = 5000; ///< ToDo -> Benchmark supereight.
    if (blocking_) {
      const bool result = supereightFrames_.PushBlockingIfFull(
          supereightFrame, supereightQueueSize);
      cvNewSupereightData_.notify_one();
    } else {
      // Push measurement and pop the oldest entry.
      const bool result = supereightFrames_.PushNonBlockingDroppingIfFull(
          supereightFrame, supereightQueueSize);
      cvNewSupereightData_.notify_one();
      if (result)
        LOG(WARNING) << "Oldest Supereight frame dropped";
    }
  }
}

bool SupereightInterface::start() {
  
  // do not remove this cout!!!
  std::cout << "\n\nStarting supereight processing... \n\n"; // segfault without this cout (??)

  timeZero_ = okvis::Time::now();

  // STart the thread that prepares the Converts the Data in Supereight
  // format
  dataPreparationThread_ =
      std::thread(&SupereightInterface::pushSuperEightData, this);

  // Start the supereight processing thread.
  processingThread_ =
      std::thread(&SupereightInterface::processSupereightFrames, this);
  return true;
}

void SupereightInterface::display() {

  // // Display the Depth frame.
  // CameraMeasurement depthMeasurement;
  // if (depthMeasurements_.getCopyOfFront(&depthMeasurement)) {
  //   cv::imshow("Depth frames", depthMeasurement.measurement.depthImage);
  // }

  // //Display from the seframes queue
  // // should work. does not. problem in loading images to seframesqueue
  // SupereightFrame SupereightFrame_;
  // if (supereightFrames_.getCopyOfFront(&SupereightFrame_)) {
  //   cv::imshow("Depth from Seframes", depthImage2Mat(SupereightFrame_.depthFrame));
  // }
}

bool SupereightInterface::stateUpdateCallback(
    const State &latestState, const TrackingState &latestTrackingState,
    const StateVector &keyframeStates) {

  // Assemble OKVIS updates in a struct and push in the corresponding Queue.
  OkvisUpdate latestStateData(latestState, keyframeStates,
                              latestState.timestamp,
                              latestTrackingState.isKeyframe);
  const size_t stateUpdateQueue = 100; ///< 5 seconds of data at 20Hz.
  if (blocking_) {
    const bool result =
        stateUpdates_.PushBlockingIfFull(latestStateData, stateUpdateQueue);
    cvNewSensorMeasurements_.notify_one();
    return result;
  } else {
    if (stateUpdates_.PushNonBlockingDroppingIfFull(latestStateData,
                                                    stateUpdateQueue)) {
      // Oldest measurement dropped
      LOG(WARNING) << "Oldest state  measurement dropped";
      cvNewSensorMeasurements_.notify_one();
      return true;
    }
  }

  return false;
}

void SupereightInterface::publishSubmaps()
{

 if (submapCallback_) 
  {
    // USE MUTEX HERE? dont think so (read only)
    std::thread publish_submaps(submapCallback_, submapPoseLookup_, submapLookup_);
    publish_submaps.detach();
  }  

  if (submapMeshesCallback_) 
  {
    // USE MUTEX HERE? dont think so (read only)
    std::thread publish_meshes(submapMeshesCallback_, submapPoseLookup_);
    publish_meshes.detach();
  }  
}

bool SupereightInterface::detectCollision(const ompl::base::State *state) 
{
  // with hash table, checking for circle around drone

  if(submapLookup_.empty()) return false;

  const ompl::base::RealVectorStateSpace::StateType *pos = state->as<ompl::base::RealVectorStateSpace::StateType>();

  // the voxel we want to query (wrt world frame, homogenous)
  Eigen::Vector4d r(pos->values[0],pos->values[1],pos->values[2],1);

  // check occ inside a sphere around the drone 
  // for now its very sparse 
  // lowest res is 1 voxel = 0.2m
  const float rad = 0.01; // radius of the sphere

  for (float z = -rad; z <= rad; z += rad/2)
  {
    for (float y = -rad; y <= rad; y += rad/2)
    {
      for (float x = -rad; x <= rad; x += rad/2)
      {
        if((std::pow(x,2) + std::pow(y,2) + std::pow(z,2)) > std::pow(rad,2)) continue; // skip if point is outside radius

        // CHECK OCCUPANCY AMONG LOCAL SUBMAPS:

        // add offset to r 
        Eigen::Vector4d r_new = r;
        r_new(0) += x;
        r_new(1) += y;
        r_new(2) += z;

        // its respective truncated coordinates (the 1x1x1 box it's in)
        Eigen::Vector3i box_coord;
        for (int i = 0 ; i < 3 ; i++)
        {
          box_coord(i) = (int)(floor(r_new(i)));
        }

        // we add occupancy values here
        double tot_occupancy = 0; 

        //to consider universally unmapped voxels as occupied;
        bool unmapped = true;

        // iterate over submap ids (only the ones that contain current state!)
        for (auto& id: hashTable_[box_coord]) {

          //std::cout << "--------- \nr " << r << "\n";
          //std::cout << "r_new " << r_new << "\n";
          //std::cout << "id " << id.first << "\n";

          // transform state coords to check from world to map frame
          const Eigen::Matrix4d T_wf = submapPoseLookup_[id].T(); // kf wrt world
          const Eigen::Vector4d r_map_hom = T_wf.inverse() * r_new;// state coordinates (homogenous) in map frame

          const Eigen::Vector3f r_map = r_map_hom.head<3>().cast<float>(); // take first 3 elems and cast to float
          


          // if voxel belongs to current submap
          if((*submapLookup_[id])->contains(r_map))
          {
            unmapped = false;
            auto data = (*submapLookup_[id])->getData(r_map);
            double occupancy = data.occupancy * data.weight; // occupancy value of the 3d point
            tot_occupancy += occupancy;
          }
          // else std::cout << "unmapped \n";
        } 
        
        // when done iterating over submaps, check total occupancy / check if unmapped

        if(tot_occupancy >= 0) {
          // std::cout << "\n \n OCCUPIED! \n \n";
          return false; // occupied!
        }

        // if the voxel is not found in any submap
        if(unmapped) {
          // std::cout << "\n \n UNMAPPED! \n \n"; 
          return false;
        }
        
      } // x
    } // y
  } // z

  // if we reach this point, it means every point on the circle is free
  return true;

}

void SupereightInterface::replan() 
{
  if(replanCallback_) {
    // calls plan() function in Planner -> replans with new start, same goal & new map
    std::thread publish(replanCallback_);
    publish.detach();
  }
}

