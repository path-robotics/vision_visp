#include <ros/package.h>
#include <geometry_msgs/Transform.h>
#include <geometry_msgs/TransformStamped.h>
#include <glugun_camera/camera_config_loader.h>
#include <string>
#include <vector>
#include <map>
#include <glugun_camera/LocateCamera.h>
#include "names.h"
#include "glugun_client.h"

using namespace glugun;

Client::Client() {
  // Client
  compute_effector_camera_quick_client_ =
    nh_.serviceClient<visp_hand2eye_calibration::compute_effector_camera_quick>
    (visp_hand2eye_calibration::compute_effector_camera_quick_service);

  // Server
  locate_camera_server_ = nh_.advertiseService("locate_camera", &Client::locate_camera_cb, this);
}

Client::~Client() {

}

bool Client::locate_camera_cb(glugun_camera::LocateCamera::Request &req,
                              glugun_camera::LocateCamera::Response &res) {

  // Get path path to 'glugun_camera' package
  std::string ros_pkg_path = ros::package::getPath("glugun_camera");
  std::string config_dir_path = ros_pkg_path + "/config/";

  // Instaniate a CameraConfigLoader object
  glugun::CameraConfigLoader loader(config_dir_path);

  // And get the base2t and cam2chess transforms
  std::vector<geometry_msgs::TransformStamped> base2t_tfs = loader.get_base2t_tfs();
  std::map<int, geometry_msgs::TransformStamped> cam2chess_tfs = loader.get_cam2chess_tfs();


  // Iterate over the found chessboard transforms and add them to the request
  visp_hand2eye_calibration::compute_effector_camera_quick emc_quick_comm;
  for (std::map<int, geometry_msgs::TransformStamped>::const_iterator it = cam2chess_tfs.begin(); it != cam2chess_tfs.end(); ++it) {
    int idx = it->first;
    // base2t
    geometry_msgs::Transform base2t_tf = base2t_tfs[idx].transform;
    geometry_msgs::Transform cam2chess_tf = it->second.transform;
    emc_quick_comm.request.camera_object.transforms.push_back(cam2chess_tf);
    emc_quick_comm.request.world_effector.transforms.push_back(base2t_tf);
  }
  // Send request to server
  if (compute_effector_camera_quick_client_.call(emc_quick_comm)) {
    ROS_INFO_STREAM("hand_camera: "<< std::endl << emc_quick_comm.response.effector_camera);
    res.success = true;
    res.message = "Calibration successful";
    res.transform.transform = emc_quick_comm.response.effector_camera;
    res.transform.header.frame_id = "t_link";
    res.transform.child_frame_id = "camera_link";
    res.transform.header.stamp = ros::Time::now();
    ROS_INFO_STREAM(res);
    return true;
  }
  else {
    res.success = false;
    res.message = "Failed to call service";
    ROS_ERROR("Failed to call service");
    return false;
  }
}
