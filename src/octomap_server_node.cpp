/**
* octomap_server: A Tool to serve 3D OctoMaps in ROS (binary and as visualization)
* (inspired by the ROS map_saver)
* @author A. Hornung, University of Freiburg, Copyright (C) 2009 - 2012.
* @see http://octomap.sourceforge.net/
* License: BSD
*/

/*
 * Copyright (c) 2009-2012, A. Hornung, University of Freiburg
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the University of Freiburg nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include <ros/ros.h>
#include <octomap_server/OctomapServer.h>
#include <dirent.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>

#define USAGE "\nUSAGE: octomap_server <map.[bt|ot]>\n" \
        "  map.bt: inital octomap 3D map file to read\n"

using namespace octomap_server;
using namespace std;

void LoadTUMPosesWithTimestamp(const std::string& filePath, vector<double>& timestamps, vector<Eigen::Matrix4f>& poses)
{
    ifstream infile(filePath, ios::in);
    string temp, temp2mat;
    while (getline(infile, temp)) {
        vector<float> pose;
        istringstream LineBand(temp);
        LineBand >> temp2mat; // timestamp
        timestamps.push_back(atof(temp2mat.data()));

        while (LineBand >> temp2mat) {
            pose.push_back(atof(temp2mat.data()));
        }

        Eigen::Matrix4f T = Eigen::Matrix4f::Identity();
        Eigen::Vector3f t(pose[0], pose[1], pose[2]);
        Eigen::Quaternionf q(pose[6], pose[3], pose[4], pose[5]);
        Eigen::Matrix3f R = q.matrix();
        T.block(0,3,3,1) = t;
        T.block(0,0,3,3) = R;
        poses.push_back(T);
    }
    infile.close();
}

void GetFilelistFromDir(std::string path, std::vector<std::string>& files, string suffex)
{
    DIR* dir;
    dir = opendir(path.c_str());
    struct dirent* ptr;
    while ((ptr = readdir(dir)) != NULL) {
        if (ptr->d_name[0] == '.') {
            continue;
        }
        if (string(ptr->d_name).find(suffex) != string::npos)
            files.push_back(ptr->d_name);
    }
    closedir(dir);
    sort(files.begin(), files.end());
}

void MappingOfflineWithPosesAndPCFrame(const std::string& tum_poses_file, const std::string& lidar_scan_dir, OctomapServer& server, ros::NodeHandle& nh)
{
    ROS_INFO("tum_poses_file: %s, lidar_scan_dir: %s", tum_poses_file.c_str(), lidar_scan_dir.c_str());
    vector<double> timestamps;
    vector<Eigen::Matrix4f> poses;
    vector<string> pcd_files;
    LoadTUMPosesWithTimestamp(tum_poses_file, timestamps, poses);
    GetFilelistFromDir(lidar_scan_dir, pcd_files, "pcd");
    ROS_INFO(" poses.size %lu and pcd_files.size %lu", poses.size(), pcd_files.size());
    const string frame_id = "rslidar";
    if(poses.size() != pcd_files.size()){
      ROS_ERROR(" poses.size is not equal to pcd_files.size.");
    }
    OctomapServer::PCLPointCloud::Ptr pc(new OctomapServer::PCLPointCloud);
    ros::Publisher trajectory_pub;
    nav_msgs::Path trajectory;
    trajectory.header.frame_id = "map";
    trajectory.header.stamp = ros::Time::now();
    geometry_msgs::PoseStamped pose;
    pose.header = trajectory.header;
    trajectory_pub = nh.advertise<nav_msgs::Path>("tum_poses_trajectory", 10);
    ros::WallTime startTime = ros::WallTime::now();
    for(size_t i = 0; i < pcd_files.size(); i++){
      pcl::io::loadPCDFile(lidar_scan_dir + "/" + pcd_files[i], *pc);
      server.insertCloudWithPose(*pc, poses[i], ros::Time(timestamps[i]), frame_id);
      pose.pose.position.x = poses[i](0, 3);
      pose.pose.position.y = poses[i](1, 3);
      pose.pose.position.z = 0.0; //poses[i](2, 3);
      trajectory.poses.push_back(pose);
      if(trajectory_pub.getNumSubscribers() > 0){
        trajectory_pub.publish(trajectory);
      }
    }
    double building_octree_map_elapsed = (ros::WallTime::now() - startTime).toSec();
    ROS_INFO("Building Map took %f sec", building_octree_map_elapsed);
}

int main(int argc, char** argv){
  ros::init(argc, argv, "octomap_server");
  ros::NodeHandle nh;
  const ros::NodeHandle private_nh("~");
  std::string mapFilename(""), mapFilenameParam(""), tum_poses_file(""), lidar_scan_dir("");

  if (argc > 2 || (argc == 2 && std::string(argv[1]) == "-h")){
    ROS_ERROR("%s", USAGE);
    exit(-1);
  }

  OctomapServer server(private_nh, nh);
  ros::spinOnce();

  if(private_nh.getParam("tum_poses_file", tum_poses_file) && private_nh.getParam("lidar_scan_dir", lidar_scan_dir)){
      MappingOfflineWithPosesAndPCFrame(tum_poses_file, lidar_scan_dir, server, nh);
      while(ros::ok()){
        ros::WallTime s = ros::WallTime::now();
        server.publishAll();
        double pub_map_elapsed = (ros::WallTime::now() - s).toSec();
        ROS_INFO("Map publishing in OctomapServer took %f sec", pub_map_elapsed);
      }
      return 0;
  }

  if (argc == 2){
    mapFilename = std::string(argv[1]);
  }

  if (private_nh.getParam("map_file", mapFilenameParam)) {
    if (mapFilename != "") {
      ROS_WARN("map_file is specified by the argument '%s' and rosparam '%s'. now loads '%s'", mapFilename.c_str(), mapFilenameParam.c_str(), mapFilename.c_str());
    } else {
      mapFilename = mapFilenameParam;
    }
  }

  if (mapFilename != "") {
    if (!server.openFile(mapFilename)){
      ROS_ERROR("Could not open file %s", mapFilename.c_str());
      exit(1);
    }
  }

  try{
    ros::spin();
  }catch(std::runtime_error& e){
    ROS_ERROR("octomap_server exception: %s", e.what());
    return -1;
  }

  return 0;
}
