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
  //inputDepth.convertTo(depthScaled, CV_32FC1, 1.f / 1000.f);
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
                                  KeyFrameDataVec &keyFrameDataVec,
                                  bool &loop_closure) {
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
  if (initialStateData.keyframeStates.empty()) return false;

  Transformation T_WS0;

  // ============ 

  // Retrieve the Pose of the current active kf (the one with most co observations)

  // if current frame is a keyframe (faster)
  // if (initialStateData.isKeyframe) {
  //   keyframeId = initialStateData.latestState.id.value();
  //   T_WS0 = initialStateData.latestState.T_WS;
  //   submapPoseLookup_.insert(std::make_pair(keyframeId, T_WS0 * T_SC_));
  //   std::cout << "New kf  " << keyframeId << "\n";
  // }
  // else, and only if submaplookup contains id (older keyframe is the one w most co obs), check inside it for pose
  // else if (submapPoseLookup_.count(initialStateData.currentKeyframe)) { 
  //   keyframeId = initialStateData.currentKeyframe;
  //   T_WS0 = submapPoseLookup_[keyframeId] * T_CS_; // express in sensor frame
  // }
  // this happens if a frame is both not a keyframe and the 
  // current kf Id cant be found in the lookup table
  // else {
  //   std::cout << "looking for submap " << initialStateData.currentKeyframe << " among " "\n";
  //   for (auto& id: submapPoseLookup_) {
  //     std::cout << id.first << "\n";
  //   }
  //   throw std::runtime_error("FECK!");
  // }

  // ============

  // simpler version. we dont integrate always in current keyframe, but in 
  // LATEST one, which might be different in some occasions but is not a problem

  // std::cout << "id " << initialStateData.latestState.id.value() << " " << initialStateData.isKeyframe << "\n";

  bool get_last_kf = false;
  if (initialStateData.isKeyframe || no_kf_yet) { // first update should always be keyframe
    no_kf_yet = false;
    T_WS0 = initialStateData.latestState.T_WS;
    keyframeId = initialStateData.latestState.id.value();
    latestKeyframeId = keyframeId;
  }
  else { 
    get_last_kf = true; // get latest kf from keyframedatavec (check if they are ordered)
    keyframeId = latestKeyframeId;
  }

  // KF poses. 
  keyFrameDataVec = KeyFrameDataVec(initialStateData.keyframeStates.size());
  for (size_t i = 0; i < initialStateData.keyframeStates.size(); i++) {
    const auto &state = initialStateData.keyframeStates[i];
    // std::cout << " " << state.id.value() << "\n";
    if(get_last_kf && (state.id.value() == latestKeyframeId)) T_WS0 = state.T_WS;
    keyFrameDataVec.at(i) = KeyframeData(state.id.value(), state.T_WS * T_SC_);
  }

  T_WC0 = T_WS0 * T_SC_;

  // Is current state a loop closure state?
  loop_closure = initialStateData.loop_closure;

  // HACK: We can use the built in OKVIS prediction. Todo -> try to avoid the
  // duplicate queues and duplicate  code and check if we can directly Query the
  // propagated pose from OKVIS

  // We are getting all imu meas from the initial state to the final state
  // put them in the deque --> later we integrate them
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
  // suppress warnings.
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

    // Proceed with supereight integration
    // const auto start_time = std::chrono::high_resolution_clock::now();

    //  Update pose lookup --> each time a new seframe arrives, it contains updated info on all kf poses.
    //  we update here all poses

    // std::cout << "optimized KFs: \n";

    for (auto &keyframeData : supereightFrame.keyFrameDataVec) {

    
      const auto id = keyframeData.id;
      const auto T_WM = keyframeData.T_WM;

      // std::cout << " " << id << "\n";
      
      // Check If Id exists.
      if (submapPoseLookup_.count(id)) {
        // Update.
        submapPoseLookup_[id] = T_WM;
      } else {
        // Insert
        submapPoseLookup_.insert(std::make_pair(id, T_WM));
      }
    }

    // if a loop closure was detected, redo hashing
    if(supereightFrame.loop_closure) {
      std::thread hashing_thread(&SupereightInterface::redoSpatialHashing, this, submapLookup_, submapPoseLookup_);
      hashing_thread.detach();
    }

    // Chech whether we need to create a new submap. --> integrate in new or existing map?
    // this is the latest "active" keyframe id
    static uint64_t prevKeyframeId = supereightFrame.keyframeId;

    // =========== Current KF has changed, and is distant enough  ===========

    // Finish up last map (hash + savemesh), create new map 

    //compute distance from last keyframe:
    bool distant_enough = false;
    const double treshold = 4.0;
    auto distance = (submapPoseLookup_[supereightFrame.keyframeId].r() - submapPoseLookup_[prevKeyframeId].r()).norm();
    if (distance > treshold) distant_enough = true;

    // current kf has changed, and it is distant enough from last one
    if ((supereightFrame.keyframeId != prevKeyframeId && distant_enough)|| submaps_.empty()) { 
      
      // hash & save map we just finished integrating
      // 4 safety, check that submap exists in lookup
      if (!submaps_.empty()) {

        std::cout << "Completed integrating submap " << prevKeyframeId << "\n";

        // do the spatial hashing (do it threaded)
        std::thread hashing_thread(&SupereightInterface::doSpatialHashing, this, prevKeyframeId, submapPoseLookup_[prevKeyframeId], submapLookup_[prevKeyframeId]);
        hashing_thread.detach();

        const std::string meshFilename = meshesPath_ + "/" + std::to_string(prevKeyframeId) + ".ply";

        (*(submapLookup_[prevKeyframeId]))->saveMesh(meshFilename);
                
        // call submap visualizer (it's threaded)
        publishSubmaps();
      }

      // create new map
      static unsigned int submap_counter = 0;
      submap_counter ++;
      std::cout << "New submap no. " << submap_counter << " (kf Id: " << supereightFrame.keyframeId << ")" << "\n";

      // Create a new submap and reset frame counter
      submaps_.emplace_back(
          new se::OccupancyMap<se::Res::Multi>(mapConfig_, dataConfig_));
      frame = 0;

      // Add the (keyframe Id, iterator) pair in the submapLookup_
      // We are adding the map that is curently being integrated (submaps back)
      submapLookup_.insert(std::make_pair(supereightFrame.keyframeId,
                                          std::prev(submaps_.end())));

      // do a preliminary hashing (allocate 10x10x10 box in hash table)
      // we do this so that we can plan even while integrating current map
      std::thread prelim_hashing_thread(&SupereightInterface::doPrelimSpatialHashing, this, supereightFrame.keyframeId, submapPoseLookup_[supereightFrame.keyframeId].r());
      prelim_hashing_thread.detach();

      // now we integrate in this keyframe, until we find a new one that is distant enough
      prevKeyframeId = supereightFrame.keyframeId;

      }

      // =========== END Current KF has changed ===========

      // Integrate in the map tied to current keyframe

      // Retrieve the active submap.
      // can use the lookup bc every time a new submap is created, its also inserted there
      auto &activeMap = *(submapLookup_[prevKeyframeId]);

      se::MapIntegrator integrator(
          *activeMap); //< ToDo -> Check how fast this constructor is
      integrator.integrateDepth(sensor_, supereightFrame.depthFrame,
                                (submapPoseLookup_[prevKeyframeId].inverse() * supereightFrame.T_WC).T().cast<float>(), frame);
      frame++;      

      // const auto current_time = std::chrono::high_resolution_clock::now();
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
    bool loop_closure;
    if (!predict(depthMeasurement.timeStamp, T_WC0, T_WC, lastKeyframeId,
                 keyFrameDataVec, loop_closure)) continue;

    // Construct Supereight Frame and push to the corresponding Queue
    const SupereightFrame supereightFrame(
        T_WC, T_WC0,
        depthMat2Image(depthMeasurement.measurement.depthImage), lastKeyframeId,
        keyFrameDataVec, loop_closure);

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
  
  std::cout << "\n\nStarting supereight processing... \n\n";

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
  // if (supereightFrames_.getCopyOfFront(&SupereightFrame_)) {
  //   cv::imshow("Depth from Seframes", depthImage2Mat(SupereightFrame_.depthFrame));
  // }
}

bool SupereightInterface::stateUpdateCallback(
    const State &latestState, const TrackingState &latestTrackingState,
    std::shared_ptr<const okvis::AlignedVector<State>> keyframeStates) {

  // Assemble OKVIS updates in a struct and push in the corresponding Queue.
  OkvisUpdate latestStateData(latestState, 
                              *keyframeStates,
                              latestState.timestamp,
                              latestTrackingState.isKeyframe,
                              latestTrackingState.currentKeyframeId.value(),
                              latestTrackingState.recognisedPlace);
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
    std::thread publish_submaps(submapCallback_, submapPoseLookup_, submapLookup_);
    publish_submaps.detach();
  }  

  if (submapMeshesCallback_) 
  {
    std::thread publish_meshes(submapMeshesCallback_, submapPoseLookup_);
    publish_meshes.detach();
  }  
}

void SupereightInterface::fixReadLookups()
{
  submapLookup_read = submapLookup_;
  submapPoseLookup_read = submapPoseLookup_;
  hashTable_read = hashTable_;
}

// dont change pass by value
void SupereightInterface::redoSpatialHashing(std::unordered_map<uint64_t, SubmapList::iterator> maplookup,
                                            std::unordered_map<uint64_t, Transformation> poselookup) 
{   
  using namespace std::chrono_literals;

  std::this_thread::sleep_for(100ms);

  hashTableMutex_.lock();

  std::cout << "Loop closure: re-assigning spatial hash table \n";

  for (auto &it : hashTableInverse_) {

    const auto id = it.first;
    
    // if index not in hash map, discard
    if (!hashTableInverse_.count(id)) continue;

    Eigen::Matrix<float,6,1> bounds = submapDimensionLookup_[id];

    Eigen::Vector3f min_box_metres = {bounds(0),bounds(1),bounds(2)};
    Eigen::Vector3f max_box_metres = {bounds(3),bounds(4),bounds(5)};
            
    // std::cout << "   Submap " << id <<  "\n";

    // remove boxes for "id" submap
    for (auto &pos : hashTableInverse_[id])
    {
      hashTable_[pos].erase(id); // remove id from box
      hashTableInverse_[id].erase(pos); // remove box from id
    }          
    
    auto T_KM = (*(maplookup[id]))->getTWM();
    auto T_WK = poselookup[id].T();
    auto T_WM = T_WK.cast<float>() * T_KM;

    const float side = 1.0; // hardcoded hash map box side of 1m
    const float step = 0.5 * side * sqrt(2); // this ensures full cover of submap space


    for (float x = min_box_metres(0); x <= max_box_metres(0); x+=step)
    {
      for (float y = min_box_metres(1); y <= max_box_metres(1); y+=step)
      {
        for (float z = min_box_metres(2); z <= max_box_metres(2); z+=step) 
        {
          
          // get offset value (this pos is in map frame)
          Eigen::Vector4f pos_map(x,y,z,1);

          // transform into world frame
          Eigen::Vector4f pos_world;
          pos_world = T_WM * pos_map;

          // floor transformed value
          Eigen::Vector3i pos_floor;
          for (int i = 0 ; i < 3 ; i++)
          {
            // if side is e.g. 2, a value of 4.5,4.5,4.5 is mapped to box 2,2,2
            pos_floor(i) = (int)(floor(pos_world(i)/side));
          }

          // std::cout << "   box \n " << pos_floor <<  "\n";

          // add to index 
          hashTable_[pos_floor].insert(id);

          // add pos to inverse index
          hashTableInverse_[id].insert(pos_floor);
        }
      }
    }          
  }

  hashTableMutex_.unlock();

}

// pass by value needed
void SupereightInterface::doPrelimSpatialHashing(const uint64_t id, const Eigen::Vector3d pos_kf)
{

  const uint64_t side = 1; // step (in metre)s of the spatial grid
  const uint64_t box_side = 10; // dim of the box we'll allocate
  
  // box dims, in metres
  Eigen::Vector3i min_box_metres = Eigen::Vector3i::Constant(-box_side);
  Eigen::Vector3i max_box_metres = Eigen::Vector3i::Constant(box_side);
  Eigen::Matrix<int,6,1> dims;
  dims << min_box_metres, max_box_metres;

  // box dims, in box units (not metres)
  Eigen::Vector3i min_box;
  Eigen::Vector3i max_box;

  // create 10x10x10m box around kf pos. round values to box units
  for (int i = 0 ; i < 3 ; i++)
  {
    min_box(i) = floor((pos_kf(i) - box_side)/side);
    max_box(i) = floor((pos_kf(i) + box_side)/side);
  }

  hashTableMutex_.lock();

  // std::cout << "PRELIM HASHING \n";

  // add dimensions in lookup
  // should be relative to map frame but who cares... this is just a big box
  // this needs to be in metres instead
  submapDimensionLookup_.insert(std::make_pair(id,dims.cast<float>()));

  // index the box, without caring about orientation.
  // its just a dumb hack to allow planning for current submap
  for (int x = min_box(0); x <= max_box(0); x+=1)
    {
      for (int y = min_box(1); y <= max_box(1); y+=1)
      {
        for (int z = min_box(2); z <= max_box(2); z+=1) 
        {
          Eigen::Vector3i pos(x,y,z);

          // add to index 
          hashTable_[pos].insert(id);

          // add pos to inverse index
          hashTableInverse_[id].insert(pos);
        }
      }
    }        

  hashTableMutex_.unlock();

}

// do not change pass by value
void SupereightInterface::doSpatialHashing(const uint64_t id, const Transformation Tf, const SubmapList::iterator map) 
{ 
  // if we already hashed this map, do nothing
  // if (hashTableInverse_.count(id)) return;

  hashTableMutex_.lock();

  // std::cout << "HASHING \n";

  // we first need to get rid of the preliminary indexing we did when 
  // creating map (we indexed a 10x10x10 box)
  if (hashTableInverse_.count(id)) 
  { // this should always be the case
    for (auto &pos : hashTableInverse_[id])
    {
      hashTable_[pos].erase(id); // remove id from box
      hashTableInverse_[id].erase(pos); // remove box from id
      submapDimensionLookup_.erase(id); // remve dimensions
    } 
  }

  hashTableMutex_.unlock();

  Eigen::Vector3i min_box(1000,1000,1000);
  Eigen::Vector3i max_box(-1000,-1000,-1000);

  // ======== get bounding box dimensions (in map frame) ========

  auto octree_ptr = (*map)->getOctree();

  auto resolution = (*map)->getRes();
  auto T_KM = (*map)->getTWM();
  auto T_WK = Tf.T();
  auto T_WM = T_WK.cast<float>() * T_KM;

  for (auto octant_it = se::LeavesIterator<OctreeT>(octree_ptr.get()); octant_it != se::LeavesIterator<OctreeT>(); ++octant_it) {
        const auto octant_ptr = *octant_it;

        // first, check if octant (node or block, dont care at this stage), is completely inside current bounds
        // get the two min and max corners, and check them against bounds
        Eigen::Vector3i corner_min = octant_ptr->getCoord(); 
        Eigen::Vector3i corner_max = corner_min + Eigen::Vector3i::Constant(se::octantops::octant_to_size<OctreeT>(octant_ptr));
        bool inside_bounds = true;
        for (int i = 0; i < 3; i++) {
          // if not inside bounds
          if (corner_min(i) < min_box(i) || corner_max(i) > max_box(i))
          {
            inside_bounds = false;
            break;
          }
        }
        // if octant is completely inside --> skip octant
        if(inside_bounds) continue;


        int node_size;
        int node_scale;
        
        // Differentiate between block and node processing
        if (octant_ptr->isBlock()) {
            // If the leaf is a block we'll need to iterate over all voxels at the current scale
            const Eigen::Vector3i block_coord = octant_ptr->getCoord();
            const BlockType* block_ptr = static_cast<const BlockType*>(octant_ptr);
            // Find current scale of the block leaves and their size
            const int node_scale = block_ptr->getCurrentScale();
            const int node_size  = 1 << node_scale;

            // iterate through voxels inside block
            for (int x = 0; x < BlockType::getSize(); x += node_size) {
                for (int y = 0; y < BlockType::getSize(); y += node_size) {
                    for (int z = 0; z < BlockType::getSize(); z += node_size) {

                      
                      // if the voxel is unobserved, skip
                      const auto data = block_ptr->getData();
                      if (data.weight == 0) continue;

                      const Eigen::Vector3i node_coord = block_coord + Eigen::Vector3i(x, y, z);
                      // std::cout << "voxel_coord \n" << node_coord << "\n";

                      
                      Eigen::Vector3i voxel_corner_min = node_coord;
                      Eigen::Vector3i voxel_corner_max = voxel_corner_min + Eigen::Vector3i(node_size, node_size, node_size);
                      for (int i = 0; i < 3; i++) {
                        // if not inside bounds, update either max or min
                        if (voxel_corner_min(i) < min_box(i))
                        {
                          // std::cout << "update min \n";
                          min_box(i) = voxel_corner_min(i);
                        }
                        if (voxel_corner_max(i) > max_box(i))
                        {
                          // std::cout << "update max \n";
                          max_box(i) = voxel_corner_max(i);
                        }
                      }

                      // ... or, just check for voxel coord. not for all corners ?
                      // faster, but with the adaptive res thing, could mean that the voxels are huge and
                      // we are actually making huge errors, not 20 cm errors

                    } // z
                } // y
            } // x
         }
        else { // if is node

            
          const auto data = static_cast<typename OctreeT::NodeType*>(octant_ptr)->getData();
          if (data.weight == 0) continue;

          node_size = static_cast<typename OctreeT::NodeType*>(octant_ptr)->getSize();

          const Eigen::Vector3i node_coord = octant_ptr->getCoord();
          
          Eigen::Vector3i node_corner_min = node_coord;
          Eigen::Vector3i node_corner_max = node_corner_min + Eigen::Vector3i(node_size, node_size, node_size);
          for (int i = 0; i < 3; i++) {
            // if not inside bounds, update either max or min
            if (node_corner_min(i) < min_box(i))
            { 
              // std::cout << "update min \n";
              min_box(i) = node_corner_min(i);
            }
            if (node_corner_max(i) > max_box(i))
            {
              // std::cout << "update max \n";
              max_box(i) = node_corner_max(i);
            }
          }

        }
    }

  Eigen::Vector3f min_box_metres;
  Eigen::Vector3f max_box_metres;
  for (int i = 0; i < 3; i++)
  {
    min_box_metres(i) = min_box(i) * resolution;
    max_box_metres(i) = max_box(i) * resolution;
  }

  // now I have the bounding box in metres, wrt the map frame.
  // this frame is separated from the real world frame by: Twk*Tkm
  // so to do hashing we must transform this box to the world frame by using this transformation
  // just like I did before with the stupid hashing. but with a double transformation

  // std::cout << "Map dimensions (meters): \nmin \n" << min_box_metres << "\nmax \n" << max_box_metres << "\n";

  // insert map bounds in the lookup
  Eigen::Matrix<float,6,1> dims;
  dims << min_box_metres, max_box_metres;
  submapDimensionLookup_.insert(std::make_pair(id,dims));

  // Check first if there already is an entry in hash table. if there is, we should delete and rewrite.
  // But to keep things simple, we just ignore that case. That happens when an older map is re-integrated
  // We only redo the hashing when a loop closure is detected.

  const float side = 1.0; // hardcoded hash map box side of 1m
  const float step = 0.5 * side * sqrt(2); // this ensures full cover of submap space

  // Eigen::Vector3i minbounds(100,100,100);
  // Eigen::Vector3i maxbounds(-100,-100,-100);

  hashTableMutex_.lock();
  
  // need to take all points -> use <=
  for (float x = min_box_metres(0); x <= max_box_metres(0); x+=step)
        {
          for (float y = min_box_metres(1); y < max_box_metres(1); y+=step)
          {
            for (float z = min_box_metres(2); z < max_box_metres(2); z+=step)
            {
              
              // get offset value (this pos is in map frame)
              Eigen::Vector4f pos_map(x,y,z,1);

              // transform into world frame
              Eigen::Vector4f pos_world;
              pos_world = T_WM * pos_map;

              // floor transformed value
              Eigen::Vector3i pos_floor;
              for (int i = 0 ; i < 3 ; i++)
              {
                // if side is e.g. 2, a value of 4.5,4.5,4.5 is mapped to box 2,2,2
                pos_floor(i) = (int)(floor(pos_world(i)/side));
              }

              // std::cout << "   box \n " << pos_floor <<  "\n";

              // add to index 
              hashTable_[pos_floor].insert(id);

              // add pos to inverse index
              hashTableInverse_[id].insert(pos_floor);
              
              // for (int i = 0; i < 3; i++) {
              //   if (pos_floor(i) < minbounds(i)) minbounds(i) = pos_floor(i);
              //   if (pos_floor(i) > maxbounds(i)) maxbounds(i) = pos_floor(i);
              // }
            }
          }
        }

  hashTableMutex_.unlock();

  // auto kf_pos = submapPoseLookup_[id].r(); 
  // std::cout << "Keyframe position: " << kf_pos(0) << " " << kf_pos(1) << " " << kf_pos(2) << "\n";
  // std::cout << "x lims: " << minbounds(0) << " | " << maxbounds(0) <<  "\n";
  // std::cout << "y lims: " << minbounds(1) << " | " << maxbounds(1) <<  "\n";
  // std::cout << "z lims: " << minbounds(2) << " | " << maxbounds(2) <<  "\n";

}



