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

  // cv::imshow("mydepth",depthMeasurement.measurement.depthImage);
  // cv::waitKey(2);

  // Push data to the Queue.
  const size_t depthQueueSize =
      100; ///< Arbitrary number. ToDo -> fine-tune once integration time has
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

bool SupereightInterface::dataReadyForProcessing() {
  // Get the timestamp of the oldest Depth frame
  CameraMeasurement oldestDepthMeasurement;
  if (!depthMeasurements_.getCopyOfFront(&oldestDepthMeasurement))
    return false;

  OkvisUpdate newestState;
  if (!stateUpdates_.getCopyOfBack(&newestState))
    return false;

  return (oldestDepthMeasurement.timeStamp <= newestState.timestamp);
}

DepthFrame SupereightInterface::depthMat2Image(const cv::Mat &inputDepth) {
  
  // need to have float values like this 1.0 = 1 mt
  // if this is not the case, do this:
  //inputDepth.convertTo(depthScaled, CV_32FC1, 1.f / 1000.f);

  // Initialise and copy
  DepthFrame output(inputDepth.size().width, inputDepth.size().height, 0.f);

  // cv::MAT and DepthFrame keep data stored in row major format.
  memcpy(output.data(), inputDepth.data,
         inputDepth.size().width * inputDepth.size().height * sizeof(float));

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
                                  Transformation &T_WC,
                                  uint64_t &keyframeId,
                                  KeyFrameDataVec &keyFrameDataVec,
                                  bool &loop_closure) {

  // Get the okvis update closest to the finalTimestamp 
  // (which is the time of the depth frame) -> then we predict with imu integration
  // Using okvis trajectory we should not need all this initialstatedata bull, 
  // but okvis trakectory does not tell us if a state is a kf and all that
  // so we still do this
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

  if (initialStateData.keyframeStates.empty()) return false;

  if (initialStateData.isKeyframe || no_kf_yet) { // first update should always be keyframe
    no_kf_yet = false;
    keyframeId = initialStateData.latestState.id.value();
    latestKeyframeId = keyframeId;
  }
  else { 
    keyframeId = latestKeyframeId;
  }

  // KF poses. 
  keyFrameDataVec = KeyFrameDataVec(initialStateData.keyframeStates.size());
  for (size_t i = 0; i < initialStateData.keyframeStates.size(); i++) {
    const auto &state = initialStateData.keyframeStates[i];
    keyFrameDataVec.at(i) = KeyframeData(state.id.value(), state.T_WS * T_SC_);
  }

  // Is current state a loop closure state?
  loop_closure = initialStateData.loop_closure;

  State propagatedState;
  if(!propagatedStates.getState(finalTimestamp, propagatedState)) throw std::runtime_error("predict"); // check that this dont happn. then just return false

  // Predicted Pose
  T_WC = propagatedState.T_WS * T_SC_;

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

    //  Update pose lookup --> each time a new seframe arrives, it contains updated info on all kf poses.
    //  we update here all poses

    for (auto &keyframeData : supereightFrame.keyFrameDataVec) {

    
      const uint64_t id = keyframeData.id;
      const Transformation T_WM = keyframeData.T_WM;
      
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
      std::vector<uint64_t> ids;
      for (auto &keyframeData : supereightFrame.keyFrameDataVec) {
        uint64_t id = keyframeData.id;
        if (!submapLookup_.count(keyframeData.id) || !submapPoseLookup_.count(keyframeData.id)) continue; 
        std::cout << "LC - Rehashing map " << id << "\n";
        std::thread hashing_thread(&SupereightInterface::redoSpatialHashing, this, id, submapPoseLookup_[id], submapLookup_[id]);
        hashing_thread.detach();
      }
    }

    // Chech whether we need to create a new submap. --> integrate in new or existing map?
    // this is the latest "active" keyframe id
    static uint64_t prevKeyframeId = supereightFrame.keyframeId;

    // =========== Current KF has changed, and is distant enough  ===========

    // Finish up last map (hash + savemesh), create new map 

    //compute distance from last keyframe:
    bool distant_enough = false;
    const double treshold = 4.0;
    const double distance = (submapPoseLookup_[supereightFrame.keyframeId].r() - submapPoseLookup_[prevKeyframeId].r()).norm();
    if (distance > treshold) distant_enough = true;

    // current kf has changed, and it is distant enough from last one
    if ((supereightFrame.keyframeId != prevKeyframeId && distant_enough)|| submaps_.empty()) { 
      
      // hash & save map we just finished integrating
      // 4 safety, check that submap exists in lookup
      if (!submaps_.empty()) {

        std::cout << "Completed integrating submap " << prevKeyframeId << "\n";

        // do the spatial hashing
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

      Eigen::Matrix4f T_KC = (submapPoseLookup_[prevKeyframeId].T().inverse() * supereightFrame.T_WC.T()).cast<float>();
      
      integrator.integrateDepth(sensor_, supereightFrame.depthFrame,
                                T_KC, frame);
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
    Transformation T_WC;
    uint64_t lastKeyframeId;
    KeyFrameDataVec keyFrameDataVec;
    bool loop_closure;
    if (!predict(depthMeasurement.timeStamp, T_WC, lastKeyframeId,
                 keyFrameDataVec, loop_closure)) continue;

    // Construct Supereight Frame and push to the corresponding Queue
    const SupereightFrame supereightFrame(
        T_WC,
        depthMat2Image(depthMeasurement.measurement.depthImage), lastKeyframeId,
        keyFrameDataVec, loop_closure);

    // Push to the Supereight Queue.
    const size_t supereightQueueSize = 500; ///< ToDo -> Benchmark supereight.
    if (blocking_) {
      supereightFrames_.PushBlockingIfFull(
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

  // Display the Depth frame.
  CameraMeasurement depthMeasurement;
  if (depthMeasurements_.getCopyOfFront(&depthMeasurement)) {
    cv::imshow("Depth", depthMeasurement.measurement.depthImage);
  }

  //Display from the seframes queue
  SupereightFrame SupereightFrame_;
  if (supereightFrames_.getCopyOfFront(&SupereightFrame_)) {
    cv::imshow("seframe", depthImage2Mat(SupereightFrame_.depthFrame));
  }
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

  std::set<StateId> affectedStateIds; // dummy, output needed by okvis trajectory
  // okvis trajectory should still work if i pass only updated keyframes (and not the whole updated states vector)
  propagatedStates.update(latestState, latestTrackingState, keyframeStates, affectedStateIds);

  if (blocking_) {
    const bool result = stateUpdates_.PushBlockingIfFull(latestStateData, stateUpdateQueue);
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
void SupereightInterface::redoSpatialHashing(const uint64_t id, const Transformation Tf, const SubmapList::iterator map) 
{   

  std::unique_lock<std::mutex> lk(hashTableMutex_);

  Eigen::Matrix4f T_KM = (*(map))->getTWM();
  Eigen::Matrix4d T_WK = Tf.T();
  Eigen::Matrix4f T_WM = T_WK.cast<float>() * T_KM;

  // sanity checks
  if (!submapDimensionLookup_.count(id) || !hashTableInverse_.count(id))
  throw std::runtime_error("redospatialhashing");

  // get map bounds
  Eigen::Matrix<float,6,1> bounds = submapDimensionLookup_[id]; 

  Eigen::Vector3f min_box_metres = {bounds(0),bounds(1),bounds(2)};
  Eigen::Vector3f max_box_metres = {bounds(3),bounds(4),bounds(5)};

  // remove boxes for "id" submap
  for (auto &pos : hashTableInverse_[id])
  {
    hashTable_[pos].erase(id); // remove id from box
    hashTableInverse_[id].erase(pos); // remove box from id
  }  

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

        // add to index 
        hashTable_[pos_floor].insert(id);

        // add pos to inverse index
        hashTableInverse_[id].insert(pos_floor);
      }
    }
  }    

  lk.unlock();

}
// pass by value needed
void SupereightInterface::doPrelimSpatialHashing(const uint64_t id, const Eigen::Vector3d pos_kf)
{

  const int side = 1; // step (in metre)s of the spatial grid
  const int box_side = 10; // dim of the box we'll allocate
  
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

  std::unique_lock<std::mutex> lk(hashTableMutex_);

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

  lk.unlock();

}

// do not change pass by value
void SupereightInterface::doSpatialHashing(const uint64_t id, const Transformation Tf, const SubmapList::iterator map) 
{ 

  Eigen::Vector3i min_box(1000,1000,1000);
  Eigen::Vector3i max_box(-1000,-1000,-1000);

  // ======== get bounding box dimensions (in map frame) ========

  auto octree_ptr = (*map)->getOctree();

  auto resolution = (*map)->getRes();
  Eigen::Matrix4f T_KM = (*map)->getTWM();
  Eigen::Matrix4d T_WK = Tf.T();
  Eigen::Matrix4f T_WM = T_WK.cast<float>() * T_KM;

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
        
        // Differentiate between block and node processing
        if (octant_ptr->isBlock()) {
            // If the leaf is a block we'll need to iterate over all voxels at the current scale
            const Eigen::Vector3i block_coord = octant_ptr->getCoord();
            const BlockType* block_ptr = static_cast<const BlockType*>(octant_ptr);
            // Find current scale of the block leaves and their size
            const int node_scale = block_ptr->getCurrentScale();
            const int node_size  = 1 << node_scale;

            // iterate through voxels inside block
            for (unsigned int x = 0; x < BlockType::getSize(); x += node_size) {
                for (unsigned int y = 0; y < BlockType::getSize(); y += node_size) {
                    for (unsigned int z = 0; z < BlockType::getSize(); z += node_size) {

                      
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

          const int node_size = static_cast<typename OctreeT::NodeType*>(octant_ptr)->getSize();

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

  std::unique_lock<std::mutex> lk(hashTableMutex_);

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

  // insert map bounds in the lookup
  Eigen::Matrix<float,6,1> dims;
  dims << min_box_metres, max_box_metres;
  submapDimensionLookup_.insert(std::make_pair(id,dims));

  // Check first if there already is an entry in hash table. if there is, we should delete and rewrite.
  // But to keep things simple, we just ignore that case. That happens when an older map is re-integrated
  // We only redo the hashing when a loop closure is detected.

  const float side = 1.0; // hardcoded hash map box side of 1m
  const float step = 0.5 * side * sqrt(2); // this ensures full cover of submap space
  
  // need to take all points -> use <=
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
              
              // for (int i = 0; i < 3; i++) {
              //   if (pos_floor(i) < minbounds(i)) minbounds(i) = pos_floor(i);
              //   if (pos_floor(i) > maxbounds(i)) maxbounds(i) = pos_floor(i);
              // }
            }
          }
        }

  lk.unlock();

}



