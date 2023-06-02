1, Choosing mapping method:

    Method 1: offline mapping with final global map 

    1. Set parameters in launch file octomap_mapping_nodelet.launch and run octomap_mapping by  
    $ roslaunch octomap_server octomap_mapping_nodelet.launch

    Method 2: offline mapping with lidar pointcloud scans and related TUM poses

    1. Set parameters in launch file octomap_mapping_nodelet_with_scan_pose.launch and run octomap_mapping by  
  
    $ roslaunch octomap_server octomap_mapping_nodelet_with_scan_pose.launch

    Method 3: online mapping with lidar mapping pointcloud scans and related TF poses

    1. Set parameters in launch file octomap_mapping_online.launch and run octomap_mapping by  
  
    $ roslaunch octomap_server octomap_mapping_online.launch

    2. Run mapping module(such as lio-sam), which should provide the TF poses
  
      Note: this method is just for visualization, for currently loop closure is not supported here. So if there occurred loop closure, the final octomap will have differ with the pointcloud map.

2, Wait ROS topic /projected_map generated. Check by  
$ rostopic echo /projected_map

3, Save /projected_map in .pgm format  
srosrun map_server map_saver -f mymap map:=/projected_map  
