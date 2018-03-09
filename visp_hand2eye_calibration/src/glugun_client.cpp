#include <ros/ros.h>
#include <ros/package.h>
#include <geometry_msgs/Transform.h>
#include <geometry_msgs/TransformStamped.h>
#include <glugun_camera/camera_config_loader.h>
#include <vector>
#include <map>
#include "visp_hand2eye_calibration/compute_effector_camera.h"
#include "visp_hand2eye_calibration/compute_effector_camera_quick.h"
#include "client.h"
#include <geometry_msgs/Transform.h>
#include "visp_hand2eye_calibration/TransformArray.h"
#include <visp_bridge/3dpose.h>
#include "names.h"

#include <visp/vpCalibration.h>
#include <visp/vpExponentialMap.h>

int main(int argc,char**argv){
  ros::init(argc, argv, "glugun_client");
  ros::NodeHandle nh;

  ros::ServiceClient compute_effector_camera_service_ =
    nh.serviceClient<visp_hand2eye_calibration::compute_effector_camera> (visp_hand2eye_calibration::compute_effector_camera_service);
  ros::ServiceClient compute_effector_camera_quick_service_ =
    nh.serviceClient<visp_hand2eye_calibration::compute_effector_camera_quick> (visp_hand2eye_calibration::compute_effector_camera_quick_service);

  visp_hand2eye_calibration::compute_effector_camera emc_comm;
  visp_hand2eye_calibration::compute_effector_camera_quick emc_quick_comm;


  // Get path path to 'glugun_camera' package
  std::string ros_pkg_path = ros::package::getPath("glugun_camera");
  std::string config_dir_path = ros_pkg_path + "/config/";

  // Instaniate a CameraConfigLoader object
  glugun::CameraConfigLoader loader(config_dir_path);

  std::vector<geometry_msgs::TransformStamped> base2t_tfs = loader.get_base2t_tfs();
  std::map<int, geometry_msgs::TransformStamped> cam2chess_tfs = loader.get_cam2chess_tfs();

  // Iterate over the found chessboard transforms
  for (std::map<int, geometry_msgs::TransformStamped>::const_iterator it = cam2chess_tfs.begin(); it != cam2chess_tfs.end(); ++it) {
    int idx = it->first;
    // base2t
    geometry_msgs::Transform base2t_tf = base2t_tfs[idx].transform;
    geometry_msgs::Transform cam2chess_tf = it->second.transform;
    emc_quick_comm.request.camera_object.transforms.push_back(cam2chess_tf);
    emc_quick_comm.request.world_effector.transforms.push_back(base2t_tf);
  }
  if (compute_effector_camera_quick_service_.call(emc_quick_comm)) {
    ROS_INFO_STREAM("hand_camera: "<< std::endl << emc_quick_comm.response.effector_camera);
  }
  else {
    ROS_ERROR("Failed to call service");
  }

  return 0 ;
}
