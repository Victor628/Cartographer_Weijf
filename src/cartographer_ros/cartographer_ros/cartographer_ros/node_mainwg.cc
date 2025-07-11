/*
 * Copyright 2016 The Cartographer Authors
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include "absl/memory/memory.h"
#include "cartographer/mapping/map_builder.h"
#include "cartographer_ros/node.h"
#include "cartographer_ros/node_options.h"
#include "cartographer_ros/ros_log_sink.h"
#include "gflags/gflags.h"
#include "tf2_ros/transform_listener.h"

#include "cartographer_ros/msg_conversion.h"

#include "ros/ros.h"
#include "cartographer_ros_msgs/FinishTrajectory.h"
#include "cartographer_ros_msgs/StartTrajectory.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "tf/tf.h"
 
void init_pose_callback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &msg);

template<typename T>
T getParam_Func(ros::NodeHandle& pnh,
                    const std::string& param_name, const T & default_val)
{
  T param_val;
  pnh.param<T>(param_name, param_val, default_val);
  return param_val;
}



DEFINE_bool(collect_metrics, false,
            "Activates the collection of runtime metrics. If activated, the "
            "metrics can be accessed via a ROS service.");
DEFINE_string(configuration_directory, "",
              "First directory in which configuration files are searched, "
              "second is always the Cartographer installation to allow "
              "including files from there.");
DEFINE_string(configuration_basename, "",
              "Basename, i.e. not containing any directory prefix, of the "
              "configuration file.");
DEFINE_string(load_state_filename, "",
              "If non-empty, filename of a .pbstream file to load, containing "
              "a saved SLAM state.");
DEFINE_bool(load_frozen_state, true,
            "Load the saved state as frozen (non-optimized) trajectories.");
DEFINE_bool(
    start_trajectory_with_default_topics, true,
    "Enable to immediately start the first trajectory with default topics.");
DEFINE_string(
    save_state_filename, "",
    "If non-empty, serialize state and write it to disk before shutting down.");

namespace cartographer_ros {
namespace {

void Run() {
  constexpr double kTfBufferCacheTimeInSeconds = 10.;
  tf2_ros::Buffer tf_buffer{::ros::Duration(kTfBufferCacheTimeInSeconds)};
  tf2_ros::TransformListener tf(tf_buffer);
  NodeOptions node_options;
  TrajectoryOptions trajectory_options;
  std::tie(node_options, trajectory_options) =
      LoadOptions(FLAGS_configuration_directory, FLAGS_configuration_basename);

  auto map_builder =
      cartographer::mapping::CreateMapBuilder(node_options.map_builder_options);
  Node node(node_options, std::move(map_builder), &tf_buffer,
            FLAGS_collect_metrics);
  auto trajectory_options_handle = &(trajectory_options);  
  ros::NodeHandle pnh("~");
  bool localization = getParam_Func<bool>(pnh, "/localization", 0);      //是否为纯定位模式
  double pos_x = getParam_Func<double>(pnh, "/set_inital_pose_x", 0.0); 
  double pos_y = getParam_Func<double>(pnh, "/set_inital_pose_y", 0.0); 
  double pos_z = getParam_Func<double>(pnh, "/set_inital_pose_z", 0.0); 
  double pos_ox = getParam_Func<double>(pnh, "/set_inital_pose_ox", 0.0); 
  double pos_oy = getParam_Func<double>(pnh, "/set_inital_pose_oy", 0.0); 
  double pos_oz = getParam_Func<double>(pnh, "/set_inital_pose_oz", 0.0); 
  double pos_ow = getParam_Func<double>(pnh, "/set_inital_pose_ow", 1.0); 
  geometry_msgs::Pose init_pose;
  init_pose.position.x = pos_x;
  init_pose.position.y = pos_y;
  init_pose.position.z = pos_z;
  init_pose.orientation.x = pos_ox;
  init_pose.orientation.y = pos_oy;
  init_pose.orientation.z = pos_oz;
  init_pose.orientation.w = pos_ow;

  if(localization == true)
  {
  //更改轨迹配置项中的初始位姿值
  *trajectory_options_handle->trajectory_builder_options.mutable_initial_trajectory_pose()->mutable_relative_pose()
    = cartographer::transform::ToProto(cartographer_ros::ToRigid3d(init_pose));
  }

  if (!FLAGS_load_state_filename.empty()) {
    node.LoadState(FLAGS_load_state_filename, FLAGS_load_frozen_state);
  }

  if (FLAGS_start_trajectory_with_default_topics) {
    node.StartTrajectoryWithDefaultTopics(trajectory_options);
  }

  ::ros::spin();

  node.FinishAllTrajectories();
  node.RunFinalOptimization();

  if (!FLAGS_save_state_filename.empty()) {
    node.SerializeState(FLAGS_save_state_filename,
                        true /* include_unfinished_submaps */);
  }
}

}  // namespace
}  // namespace cartographer_ros


//位置初始化
int traj_id = 1;
void init_pose_callback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &msg)
{
    double x = msg->pose.pose.position.x;
    double y = msg->pose.pose.position.y;
    double theta = tf::getYaw(msg->pose.pose.orientation);
    ros::NodeHandle nh;
 
    ros::ServiceClient client_traj_finish = nh.serviceClient<cartographer_ros_msgs::FinishTrajectory>("finish_trajectory");
    cartographer_ros_msgs::FinishTrajectory srv_traj_finish;
    srv_traj_finish.request.trajectory_id = traj_id;
    if (client_traj_finish.call(srv_traj_finish))
    {
        ROS_INFO("Call finish_trajectory %d success!", traj_id);
    }
    else
    {
        ROS_INFO("Failed to call finish_trajectory service!");
    }
 
    ros::ServiceClient client_traj_start = nh.serviceClient<cartographer_ros_msgs::StartTrajectory>("start_trajectory");
    cartographer_ros_msgs::StartTrajectory srv_traj_start;
    srv_traj_start.request.configuration_directory = "/home/ximei/robot_robot/src/cartographer_ros/cartographer_ros/configuration_files";//.lua文件所在路径
    srv_traj_start.request.configuration_basename = "backpack_2d_localization.lua";//lua 文件名
    srv_traj_start.request.use_initial_pose = 1;
    srv_traj_start.request.initial_pose = msg->pose.pose;
    srv_traj_start.request.relative_to_trajectory_id = 0;
    printf("&&&&&: %f__%f\n",srv_traj_start.request.initial_pose.position.x,srv_traj_start.request.initial_pose.position.y);
    if (client_traj_start.call(srv_traj_start))
    {
        // ROS_INFO("Status ", srv_traj_finish.response.status)
        ROS_INFO("Call start_trajectory %d success!", traj_id);
        traj_id++; //ID自加
    }
    else
    {
        ROS_INFO("Failed to call start_trajectory service!");
    }
}




int main(int argc, char** argv) {
  google::InitGoogleLogging(argv[0]);
  google::ParseCommandLineFlags(&argc, &argv, true);

  CHECK(!FLAGS_configuration_directory.empty())
      << "-configuration_directory is missing.";
  CHECK(!FLAGS_configuration_basename.empty())
      << "-configuration_basename is missing.";


  ::ros::init(argc, argv, "cartographer_node");
  ::ros::NodeHandle nh;
  ::ros::start();
  ros::Subscriber _pose_init_sub = nh.subscribe("/initialpose", 1, init_pose_callback);
  cartographer_ros::ScopedRosLogSink ros_log_sink;
  cartographer_ros::Run();
  ::ros::shutdown();
}
