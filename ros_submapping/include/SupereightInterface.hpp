#ifndef INCLUDE_SUPEREIGHTINTERFACE_HPP_
#define INCLUDE_SUPEREIGHTINTERFACE_HPP_

#include <boost/functional/hash.hpp>
#include <chrono>
#include <condition_variable>
#include <functional>
#include <okvis/FrameTypedefs.hpp>
#include <okvis/Measurements.hpp>
#include <okvis/ViInterface.hpp>
#include <okvis/ViParametersReader.hpp>
#include <okvis/ceres/ImuError.hpp>
#include <okvis/kinematics/Transformation.hpp>
#include <okvis/threadsafe/ThreadsafeQueue.hpp>
#include <se/supereight.hpp>
#include <thread>

// to use ompl classes
#include <ompl/base/State.h>
#include <ompl/base/spaces/SE3StateSpace.h>

// Some convenient typedefs
typedef se::Image<float> DepthFrame;
typedef okvis::CameraMeasurement CameraMeasurement;
typedef okvis::ImuMeasurement ImuMeasurement;
typedef okvis::threadsafe::ThreadSafeQueue<ImuMeasurement> ImuQueue;
typedef okvis::threadsafe::ThreadSafeQueue<CameraMeasurement> DepthFrameQueue;
typedef okvis::StateId StateId;
typedef okvis::kinematics::Transformation Transformation;
typedef okvis::TrackingState TrackingState;
typedef okvis::AlignedVector<okvis::State> StateVector;
typedef okvis::State State;
typedef se::Octree<se::Data<se::Field::Occupancy, se::Colour::Off, se::Semantics::Off>, se::Res::Multi, 8> OctreeT;
typedef typename OctreeT::BlockType BlockType;

struct OkvisUpdate {
  State latestState;
  StateVector keyframeStates;
  okvis::Time timestamp;
  uint64_t currentKeyframe;
  bool isKeyframe;
  bool loop_closure;
  OkvisUpdate(const State &latestState = State(),
              const StateVector &keyframeStates = StateVector(),
              const okvis::Time &timestamp = okvis::Time(),
              const bool isKeyframe = false,
              const uint64_t currentKeyframe = 1,
              const bool &loop_closure = false)
      : latestState(latestState), keyframeStates(keyframeStates),
        timestamp(timestamp), isKeyframe(isKeyframe), 
        currentKeyframe(currentKeyframe), loop_closure(loop_closure){};
};

typedef okvis::threadsafe::ThreadSafeQueue<OkvisUpdate> StateUpdatesQueue;

struct KeyframeData {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  uint64_t id;
  Transformation T_WM;
  KeyframeData(const uint64_t &id = 0,
               const Transformation &T_WM = Transformation::Identity())
      : id(id), T_WM(T_WM){};
};

typedef std::vector<KeyframeData, Eigen::aligned_allocator<KeyframeData>>
    KeyFrameDataVec;

/**
 * @brief Contains the data required for a single supereight map integration
 * step. each seframe contains the entire list of keyframes with updated poses
 *
 */
struct SupereightFrame {
  Transformation T_WC;
  DepthFrame depthFrame;
  uint64_t keyframeId; // id of current kf
  KeyFrameDataVec keyFrameDataVec;
  bool loop_closure;

  SupereightFrame(const Transformation &T_WC = Transformation::Identity(),
                  const DepthFrame &depthFrame = DepthFrame(640, 480, 0.f),
                  const uint64_t &keyframeId = 0,
                  const KeyFrameDataVec &keyFrameDataVec = KeyFrameDataVec{},
                  const bool &loop_closure = false)
      : T_WC(T_WC), depthFrame(depthFrame), keyframeId(keyframeId),
        keyFrameDataVec(keyFrameDataVec), loop_closure(loop_closure){};
};

typedef okvis::threadsafe::ThreadSafeQueue<SupereightFrame>
    SupereightFrameQueue;

// see include/se/map/map.hpp
typedef std::list<std::shared_ptr<se::OccupancyMap<se::Res::Multi>>> SubmapList;

// submap callback typedefs
typedef std::function<void(std::unordered_map<uint64_t, Transformation>)> submapMeshesCallback;

typedef std::function<void(std::unordered_map<uint64_t, Transformation>, std::unordered_map<uint64_t, SubmapList::iterator>)> submapCallback;

typedef std::function<void(Eigen::Vector3f)> StartStateCallback;

class SupereightInterface {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  SupereightInterface() = delete;

  /**
   * @brief      Constructs a new instance.
   *
   * @param[in]  filename  The configuration filename
   */
  SupereightInterface(const se::PinholeCameraConfig &cameraConfig,
                      const se::MapConfig &mapConfig,
                      const se::OccupancyDataConfig &dataConfig,
                      const Eigen::Matrix4d &T_SC,
                      const std::string &meshesPath)
      : T_SC_(T_SC), T_CS_(T_SC.inverse()), sensor_(cameraConfig), mapConfig_(mapConfig),
        dataConfig_(dataConfig), meshesPath_(meshesPath) {

    se::OccupancyMap<se::Res::Multi> map(mapConfig_, dataConfig_);
    loop_closure_redo_hashing = false;
  };

  /**
   * @brief      Destroys the object.
   */
  ~SupereightInterface() {
    // Shutdown all the Queues.
    imuMeasurements_.Shutdown();
    depthMeasurements_.Shutdown();
    stateUpdates_.Shutdown();
    supereightFrames_.Shutdown();

    // Wait for threads
    processingThread_.join();
    dataPreparationThread_.join();
  };

  /**
   * @brief      Adds a depth frame to the measurement Queue.
   *
   * @param[in]  stamp       The timestamp
   * @param[in]  depthFrame  The depth frame
   *
   * @return     True if successful
   */
  bool addDepthImage(const okvis::Time &stamp, const cv::Mat &depthFrame);

  /**
   * @brief      Adds an imu measurement to the measurement Queue.
   *
   * @param[in]  stamp  The timestamp
   * @param[in]  alpha  The measured linear acceleration
   * @param[in]  omega  The measured angular velocity
   *
   * @return     True if successful
   */
  bool addImuMeasurement(const okvis::Time &stamp, const Eigen::Vector3d &alpha,
                         const Eigen::Vector3d &omega);

  /**
   * @brief      Displays the most recent Depth frame.
   */
  void display();

  /**
   * @brief      Starts the processing and data prepatation threads
   *
   * @return     True when successful
   */
  bool start();

  /**
   * @brief      Stores the state and keyframe updates provided by OKVIS
   *
   * @param[in]  latestState          The current OKVIS state
   * @param[in]  latestTrackingState  The current tracking state
   * @param[in]  keyframeStates       The state of the updated Keyframes
   *
   * @return     True when successful
   */
  bool stateUpdateCallback(const State &latestState,
                           const TrackingState &latestTrackingState,
                           std::shared_ptr<const okvis::AlignedVector<State>> keyframeStates);

  /**
   * @brief      Gets the size of the to-be-processed supereight frames.
   *
   * @return     The supereight queue size.
   */
  size_t getSupereightQueueSize() { return supereightFrames_.Size(); };

  /**
   * @brief      Set blocking/not blocking mode
   *
   */
  void setBlocking(bool blocking) {
  blocking_ = blocking;
}


/**
   * @brief      Ompl planning helper. Checks state collision in the submap list
   *
   */
bool detectCollision(const ompl::base::State *state);


/**
   * @brief      triggers copy assignment of read-only lookup tables for the collision function
   *
   */
void fixReadLookups();

/**
   * @brief      Sets function inside planner class that updates start state
   *
   */
void setPlannerStartStateCallback(const StartStateCallback &startStateCallback) {startStateCallback_ = startStateCallback;}

/**
   * @brief      Set function that handles submaps processing (use this to publish in ROS)
   *
   */
void setSubmapMeshesCallback(const submapMeshesCallback &submapMeshesCallback) { submapMeshesCallback_ = submapMeshesCallback; }

/**
   * @brief      Set function that handles submaps processing (use this to publish in ROS)
   *
   */
void setSubmapCallback(const submapCallback &submapCallback) { submapCallback_ = submapCallback; }

/**
   * @brief      Visualize submaps in Publisher
   *
   */
void publishSubmaps();


// to hash a 3 int eigen vector
struct SpatialHasher {
  std::size_t operator()(const Eigen::Vector3i& a) const {
      std::size_t h = 0;

      // taken from matthias teschner 2003 collision
      const int p1 = 73856093;
      const int p2 = 19349663;
      const int p3 = 83492791;
      h = a[0]*p1 ^ a[1]*p2 ^ a[2]*p3;

      return h;
  }   
};

// To access maps
std::unordered_map<uint64_t, SubmapList::iterator> submapLookup_; // use this to access submaps (index,submap)
std::unordered_map<uint64_t, Transformation> submapPoseLookup_; // use this to access submap poses (index,pose)
std::unordered_map<uint64_t, Eigen::Matrix<float,6,1>> submapDimensionLookup_; // use this when reindexing maps on loop closures (index,dims)
// spatial hash maps --> 1x1x1 boxes (maybe make them bigger)
std::unordered_map<Eigen::Vector3i, std::unordered_set<int>, SpatialHasher> hashTable_; // a hash table for quick submap access (box coord, list of indexes)
std::unordered_map<int, std::unordered_set<Eigen::Vector3i, SpatialHasher>> hashTableInverse_; // inverse access hash table (index, list of box coords)
// Static lookups to use for collision checking (TODO make const?)
std::unordered_map<uint64_t, SubmapList::iterator> submapLookup_read; 
std::unordered_map<uint64_t, Transformation> submapPoseLookup_read;
std::unordered_map<Eigen::Vector3i, std::unordered_set<int>, SpatialHasher> hashTable_read;


private:
  /**
   * @brief      Converts an OpenCV Mat depth frame following the TUM convention
   * (5000.f -> 1m ) into the depth Images used in supereight
   *
   * @param[in]  inputDepth  The input OpenCV Mat depth
   *
   * @return     The depth frame as a supereight Image
   */
  static DepthFrame depthMat2Image(const cv::Mat &inputDepth);

  /**
   * @brief      Converts a supereight depth Image into an OpenCV Mat following
   * the TUM convention
   *
   * @param[in]  depthFrame  The supereight depth image
   *
   * @return     Depth Frame as an OpenCV Mat
   */
  static cv::Mat depthImage2Mat(const DepthFrame &depthFrame);

  bool predict(const okvis::Time &finalTimestamp, Transformation &T_WC0,
               Transformation &T_WC, uint64_t &keyframeId,
               KeyFrameDataVec &keyFrameDataVec,
               bool &loop_closure);

  /**
   * @brief      Main function of the processing thread. It integrates the
   * assembled supereigh frames (i.e. depth + pose) and creates a new submap
   * when required
   */
  void processSupereightFrames();

  /**
   * @brief      Assembles and pushes supereight frames (i.e. depth + pose) when
   * data is available.
   */
  void pushSuperEightData();

  /**
   * @brief      Checks whether we can construct a new supereight frame (depth
   * frame + pose)
   *
   * @return     True when a new supereight frame can be constructed using the
   * data in the measurement Queues.
   */
  bool dataReadyForProcessing();

  /**
   * @brief   Re assign spatial hash table for kfs involved in loop closure
   */
  void redoSpatialHashing(const KeyFrameDataVec & KeyFrameDataVec_);

  /**
   * @brief   Assign spatial hash table
   */
  void doSpatialHashing(const uint64_t & id);

  const Transformation
      T_SC_; ///< Transformation of the depth camera frame wrt IMU sensor frame
  const Transformation
      T_CS_; ///< 
  const std::string meshesPath_;  ///< Path to save the meshes
  se::PinholeCamera sensor_;      ///< Depth sensor used in supereight
  se::MapConfig mapConfig_;       ///< Supereight Map config
  se::OccupancyDataConfig dataConfig_; ///< Supereight Data config
  ImuQueue imuMeasurements_;      ///< Queue with the buffered IMU measurements
  DepthFrameQueue
      depthMeasurements_; ///< Queue with the buffered Depth measurements
  StateUpdatesQueue
      stateUpdates_; ///< Queue containing all the state updates from okvis
  SupereightFrameQueue
      supereightFrames_; ///< Queue with the s8 frames (i.e. poses and depth
                         ///< frames) to be processed
  std::condition_variable
      cvNewSensorMeasurements_; ///< Used to notify when new data(e.g. IMU,
                                ///< depth, state updates) are inserted in the
                                ///< corresponding Queues.
  std::condition_variable
      cvNewSupereightData_; ///< Used to notify when s8 frames are ready for
                            ///< integration
  std::mutex cvMutex_;
  std::mutex s8Mutex_;
  std::mutex subMapMutex_;

  std::thread processingThread_;      ///< Thread running processing loop.
  std::thread dataPreparationThread_; ///< Thread running data preparation loop.
  SubmapList submaps_;                ///< List containing all the submaps

  okvis::Time timeZero_;

  // callbacks
  submapMeshesCallback submapMeshesCallback_; // to visualize in Publisher
  submapCallback submapCallback_; // to visualize in Publisher
  StartStateCallback startStateCallback_; // to pudate start state in Planner



  // flags
  // this flag is set when we get a loop closure frame, and lowered whenever we reassign submap hashes
  bool loop_closure_redo_hashing;
  bool blocking_ = false;

};

#endif /* INCLUDE_SUPEREIGHTINTERFACE_HPP_ */
