/*
 * Copyright 2019 The Cartographer Authors
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

#include <iostream>
#include <string>
#include <vector>

//#include "absl/strings/str_cat.h"
#include "cartographer/transform/transform.h"
#include "cartographer_ros/msg_conversion.h"
#include "cartographer_ros/time_conversion.h"
#include "geometry_msgs/TransformStamped.h"


#include "cartographer/io/proto_stream.h"
#include "cartographer/io/proto_stream_deserializer.h"
#include "cartographer/io/submap_painter.h"
#include "cartographer/mapping/2d/probability_grid.h"
#include "cartographer/mapping/2d/submap_2d.h"
#include "cartographer/mapping/3d/submap_3d.h"
#include "cartographer/mapping/proto/pose_graph.pb.h"
#include "cartographer/mapping/proto/serialization.pb.h"
#include "cartographer/mapping/proto/submap.pb.h"
#include "cartographer/mapping/proto/trajectory_builder_options.pb.h"
#include "cartographer_ros/ros_map.h"
#include "cartographer_ros/submap.h"
#include "rosbag/bag.h"
#include "tf2_msgs/TFMessage.h"
#include "gflags/gflags.h"
#include "glog/logging.h"

#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>

DEFINE_string(pbstream_file, "", "pbstream file to process");
DEFINE_string(pcd_file, "", "pcd file to process");
DEFINE_string(output_file, "", " file to write to.");
DEFINE_string(parent_frame, "map", "Frame id to use as parent frame.");

namespace cartographer_ros {
namespace {

geometry_msgs::TransformStamped ToTransformStamped(
    int64_t timestamp_uts, const std::string& parent_frame_id,
    const std::string& child_frame_id,
    const cartographer::transform::proto::Rigid3d& parent_T_child) 
{
  static int64_t seq = 0;
  geometry_msgs::TransformStamped transform_stamped;
  transform_stamped.header.seq = ++seq;
  transform_stamped.header.frame_id = parent_frame_id;
  transform_stamped.header.stamp = cartographer_ros::ToRos(
      ::cartographer::common::FromUniversal(timestamp_uts));
  transform_stamped.child_frame_id = child_frame_id;
  transform_stamped.transform = cartographer_ros::ToGeometryMsgTransform(
      ::cartographer::transform::ToRigid3(parent_T_child));
  return transform_stamped;
}

void output_pbstream_trajectories(const std::string& pbstream_filename,
                                  const std::string& pcd_file,
                                  const std::string& output_filename,
                                  const std::string& parent_frame_id) 
{
  ::cartographer::io::ProtoStreamReader reader(FLAGS_pbstream_file);
  ::cartographer::io::ProtoStreamDeserializer deserializer(&reader);

  const auto& pose_graph = deserializer.pose_graph();
//   const auto pose_graph =
//       cartographer::io::DeserializePoseGraphFromFile(FLAGS_input);
  std::ofstream out_path, out_pcd_path;
  out_path.open(output_filename);
  out_pcd_path.open(pcd_file, std::ios_base::app);
    
  if (out_pcd_path.is_open()) 
  {
    out_pcd_path << "# .PCD v0.7 - Point Cloud Data file format"
    << "\n" << "VERSION 0.7"
    << "\n" << "FIELDS x y z"
    << "\n" << "SIZE 4 4 4"
    << "\n" << "TYPE F F F"
    << "\n" << "COUNT 1 1 1"
    << "\n" << "WIDTH 5000"
    << "\n" << "HEIGHT 1"
    << "\n" << "VIEWPOINT 0 0 0 1 0 0 0"
    << "\n" << "POINTS 5000" 
    << "\n" << "DATA ascii" << "\n";
  }
  Eigen::Matrix4f Twv = Eigen::Matrix4f::Identity();
  
  for (const auto trajectory : pose_graph.trajectory()) 
  {
//     const auto child_frame_id = 
//         absl::StrCat("trajectory_", trajectory.trajectory_id());
    std::stringstream child_frame_id;
    child_frame_id << "trajectory_" << trajectory.trajectory_id();
    LOG(INFO)
        << "Writing tf and geometry_msgs/TransformStamped for trajectory id "
        << trajectory.trajectory_id() << " with " << trajectory.node_size()
        << " nodes.";
    for (const auto& node : trajectory.node()) 
    {
      tf2_msgs::TFMessage tf_msg;
      geometry_msgs::TransformStamped transform_stamped = ToTransformStamped( node.timestamp(), parent_frame_id, child_frame_id.str(), node.pose() );
      tf_msg.transforms.push_back(transform_stamped);
      std::cout << "time is: " << transform_stamped.header.stamp << std::endl;
      
      Eigen::Quaternionf quaternion = Eigen::Quaternionf(transform_stamped.transform.rotation.w, transform_stamped.transform.rotation.x, transform_stamped.transform.rotation.y, transform_stamped.transform.rotation.z); // w x y z
            
      Twv.block<3, 3>(0, 0) = quaternion.toRotationMatrix();
      Twv.block<3, 1>(0, 3) << transform_stamped.transform.translation.x , transform_stamped.transform.translation.y, transform_stamped.transform.translation.z;
      
      out_path << transform_stamped.header.stamp << " " << Twv(0,0) << " " <<  Twv(0,1) << " " << Twv(0,2) << " " << Twv(0,3) 
                                                    << " " <<  Twv(1,0) << " " <<  Twv(1,1) << " " << Twv(1,2) << " " << Twv(1,3) 
                                                    << " " << Twv(2,0) << " " <<  Twv(2,1) << " " << Twv(2,2) << " " << Twv(2,3) 
                                                    << " " << Twv(3,0) << " " <<  Twv(3,1) << " " << Twv(3,2) << " " << Twv(3,3) 
                                                    << std::endl;
      
       out_pcd_path << std::setprecision(12) << transform_stamped.transform.translation.x << " " << transform_stamped.transform.translation.y << " " << transform_stamped.transform.translation.z << "\n";

    }
  }
  
  out_pcd_path.close();
  out_path.close();
}

}  // namespace
}  // namespace cartographer_ros

int main(int argc, char* argv[]) 
{
  FLAGS_alsologtostderr = true;
  google::InitGoogleLogging(argv[0]);
  google::SetUsageMessage(
      "\n\n"
      "Extracts all trajectories from the pbstream and creates a bag file with "
      "the trajectory poses stored in /tf.\nAdditionally, each trajectory is "
      "also written separately to a geometry_msgs/TransformStamped topic named "
      "after the TF child_frame_id of the trajectory.\n For each trajectory, "
      "the tool will write transforms with the tf parent_frame_id set "
      "according to the `parent_frame` commandline flag and child_frame_id to "
      "`trajectory_i`, with `i` corresponding to the `trajectory_id`.");
  google::ParseCommandLineFlags(&argc, &argv, true);
  CHECK(!FLAGS_pbstream_file.empty()) << "-pbstream_file pbstream is missing.";
  CHECK(!FLAGS_pcd_file.empty()) << "-pcd_file pcd file is missing.";
  CHECK(!FLAGS_output_file.empty()) << "-output_file is missing.";

  cartographer_ros::output_pbstream_trajectories(FLAGS_pbstream_file, FLAGS_pcd_file, FLAGS_output_file, FLAGS_parent_frame);
  return 0;
}
