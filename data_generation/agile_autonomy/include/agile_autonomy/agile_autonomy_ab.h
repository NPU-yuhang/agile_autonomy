#pragma once

#include <cv_bridge/cv_bridge.h>
#include <Eigen/Core>
#include <fstream>
#include <mutex>
#include <random>
#include <shared_mutex>

#include "agile_autonomy_msgs/Bspline.h"
#include "agile_autonomy_msgs/MultiPolyCoeff.h"
#include "agile_autonomy_msgs/MultiTrajectory.h"
#include "agile_autonomy_msgs/PositionCommand.h"
#include "agile_autonomy_utils/logging.h"
#include "agile_autonomy_utils/trajectory_ext.h"
#include "agile_autonomy_utils/visualize.h"
#include "autopilot/autopilot_helper.h"
#include "nav_msgs/Odometry.h"
#include "quadrotor_common/trajectory.h"
#include "rpg_quadrotor_msgs/Trajectory.h"
#include "ros/ros.h"
#include "rpg_common/pose.h"
#include "rpg_mpc/mpc_controller.h"
#include "sensor_msgs/Image.h"
#include "sgm_gpu/sgm_gpu.h"
#include "state_predictor/state_predictor.h"
#include "std_msgs/Bool.h"
#include "std_msgs/Empty.h"
#include "std_msgs/Float32.h"
#include "std_msgs/String.h"
#include "visualization_msgs/MarkerArray.h"

#include "avoid_msgs/TaskState.h"
#include "avoid_msgs/PointCloudInfo.h"

namespace agile_autonomy_ab {
class AgileAutonomyAB {
public:
  AgileAutonomyAB(const ros::NodeHandle& nh, const ros::NodeHandle& pnh);
  AgileAutonomyAB() : AgileAutonomyAB(ros::NodeHandle(), ros::NodeHandle("~")) {}
private:

  enum class StateMachine {
    kOff,
    kAutopilot,
    kExecuteExpert,
    kNetwork,
    kComputeLabels
  };

  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;

  logging::Logging logging_helper_;

  std::shared_ptr<visualizer::Visualizer> visualizer_;
  ros::Subscriber odometry_sub_;
  ros::Subscriber setup_logging_sub_;
  ros::Subscriber traj_sub_;
  ros::Subscriber land_sub_;
  ros::Subscriber off_sub_;
  ros::Subscriber start_flying_sub_;
  ros::Subscriber force_hover_sub_;
  ros::Subscriber completed_global_plan_sub_;
  ros::Subscriber waypoint_sub_;
  ros::Subscriber rgb_sub_;
  ros::Subscriber depth_sub_;

  ros::Publisher control_command_pub_;
  ros::Publisher ref_progress_pub_;
  ros::Publisher start_flying_pub_;
  ros::Publisher mission_stop_pub_;
  ros::Publisher setpoint_pub_;
  ros::Publisher compute_global_path_pub_;
  ros::Publisher pc_path_pub_;;

  ros::Timer save_timer_;

  void computeManeuver(const bool only_expert);
  void setupLoggingCallback(const std_msgs::BoolConstPtr& msg);
  void stopFlyingCallback(const std_msgs::BoolConstPtr& msg);
  void odometryCallback(const nav_msgs::OdometryConstPtr& msg);
  void saveLoop(const ros::TimerEvent& time);
  void wayPointCallback(const nav_msgs::PathConstPtr& msg);
  void depthCallback(const sensor_msgs::ImageConstPtr& msg);
  void saveDepth(const std::string curr_data_dir, const int frame_counter);

  quadrotor_common::QuadStateEstimate getPredictedStateEstimate(
      const ros::Time& time, const state_predictor::StatePredictor* predictor);

  void trajectoryCallback(const agile_autonomy_msgs::MultiTrajectoryConstPtr& msg);
  void landCallback(const std_msgs::EmptyConstPtr& msg);
  void offCallback(const std_msgs::EmptyConstPtr& msg);
  void forceHoverCallback(const std_msgs::EmptyConstPtr& msg);

  void publishControlCommand(const quadrotor_common::ControlCommand& control_command);
  double yawFromQuaternion(const Eigen::Quaterniond& q);
  bool selectBestNetworkPrediction(
      const std::vector<quadrotor_common::Trajectory>& nw_trajectories,
      const Eigen::Vector3d& start_pos, const Eigen::Quaterniond& start_att,
      quadrotor_common::Trajectory* const selected_trajectory);
  void completedGlobalPlanCallback(const std_msgs::BoolConstPtr& msg);
  bool convertTrajectoriesToWorldFrame(
      const std::vector<quadrotor_common::Trajectory>& nw_trajectories,
      const rpg::Pose& T_W_S, const TrajectoryExt& prev_ref,
      std::vector<quadrotor_common::Trajectory>* world_trajectories,
      std::vector<double>* trajectory_costs);
      
  bool loadParameters();
  quadrotor_common::Trajectory acrobatic_trajectory_;
  int reference_progress_abs_;

  bool save_network_trajectories_ = true;

  FrameID nw_predictions_frame_id_;
  double test_time_velocity_;
  double test_time_max_z_;
  double test_time_min_z_;

  double ctrl_cmd_delay_;

  TrajectoryExt network_prediction_;
  ros::Time time_received_prediction_;
  bool received_network_prediction_ = false;
  static constexpr int viz_id_start_ = 10;
  int viz_id_ = viz_id_start_;
  int num_traj_viz_;
  std::vector<double> sample_times_;
  std::vector<double> fine_sample_times_;

  int rollout_counter_ = 0;
  double length_straight_;
  double maneuver_velocity_;
  double save_freq_;

  bool perform_global_planning_;

  unsigned int traj_len_;
  double traj_dt_;
  int continuity_order_;

  bool enable_yawing_ = false;
  bool reference_ready_ = false;

  std::mutex nw_pred_mtx_;
  std::mutex odom_mtx_;
  std::mutex curr_ref_mtx_;
  std::mutex img_mtx;

  std::string data_dir_;
  std::string curr_data_dir_;

  quadrotor_common::TrajectoryPoint curr_reference_;
  quadrotor_common::Trajectory prev_ref_traj_;
  ros::Time t_prev_ref_traj_;
  Eigen::Vector3d goal;

  rpg_mpc::MpcController<double> base_controller_ =
      rpg_mpc::MpcController<double>(ros::NodeHandle(), ros::NodeHandle("~"), "vio_mpc_path");
  rpg_mpc::MpcParams<double> base_controller_params_;
  state_predictor::StatePredictor state_predictor_;

  StateMachine state_machine_ = StateMachine::kAutopilot;
  ros::Time time_start_logging_;
  quadrotor_common::QuadStateEstimate received_state_est_;
  double traj_sampling_freq_ = 50.0;
  double cam_pitch_angle_ = 0.0;

  int frame_counter_ = 0;
  int pred_traj_idx_ = 0;

  bool only_expert_ = false;
  bool setup_done_ = false;

  //image
  cv::Mat depth;
  cv::Mat rpg;
};
}