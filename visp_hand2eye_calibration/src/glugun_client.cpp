#include <ros/package.h>
#include <geometry_msgs/Transform.h>
#include <geometry_msgs/TransformStamped.h>
#include <glugun_camera/camera_config_loader.h>
#include <string>
#include <vector>
#include <map>
#include <Eigen/Geometry>
#include <Eigen/Eigenvalues>
#include <tf2_eigen/tf2_eigen.h>
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
  bool success = true;

  // Get path path to 'glugun_camera' package
  std::string ros_pkg_path = ros::package::getPath("glugun_camera");
  std::string config_dir_path = ros_pkg_path + "/config/";

  // Instaniate a CameraConfigLoader object
  glugun::CameraConfigLoader loader(config_dir_path);

  // And get the base2t and cam2chess transforms
  std::vector<geometry_msgs::TransformStamped> base2t_tfs = loader.get_base2t_tfs();
  std::map<int, geometry_msgs::TransformStamped> cam2chess_tfs = loader.get_cam2chess_tfs();

  // Iterate over the found chessboard transforms and add them to the request
  visp_hand2eye_calibration::compute_effector_camera_quick srv_t2cam;
  for (std::map<int, geometry_msgs::TransformStamped>::const_iterator it = cam2chess_tfs.begin(); it != cam2chess_tfs.end(); ++it) {
    int idx = it->first;
    // base2t
    geometry_msgs::Transform base2t_tf = base2t_tfs[idx].transform;
    geometry_msgs::Transform cam2chess_tf = it->second.transform;
    srv_t2cam.request.camera_object.transforms.push_back(cam2chess_tf);
    srv_t2cam.request.world_effector.transforms.push_back(base2t_tf);
  }
  // Send request to server
  if (compute_effector_camera_quick_client_.call(srv_t2cam)) {
    ROS_INFO_STREAM("hand_camera: "<< std::endl << srv_t2cam.response.effector_camera);
    res.success = true;
    res.message = "Calibration successful";
    res.t2cam.transform = srv_t2cam.response.effector_camera;
    res.t2cam.header.frame_id = "t_link";
    res.t2cam.child_frame_id = "camera_link";
    res.t2cam.header.stamp = ros::Time::now();
    ROS_INFO_STREAM(res);
  }
  else {
    res.success = false;
    res.message = "Failed to call service";
    ROS_ERROR("Failed to call service");
    success &= false;
  }

  // Iterate over the found chessboard transforms and add them to the request
  visp_hand2eye_calibration::compute_effector_camera_quick srv_chess2base;
  for (std::map<int, geometry_msgs::TransformStamped>::const_iterator it = cam2chess_tfs.begin(); it != cam2chess_tfs.end(); ++it) {
    int idx = it->first;
    // base2t
    geometry_msgs::Transform base2t_tf = base2t_tfs[idx].transform;
    geometry_msgs::Transform cam2chess_tf = it->second.transform;
    srv_chess2base.request.camera_object.transforms.push_back(base2t_tf);
    srv_chess2base.request.world_effector.transforms.push_back(cam2chess_tf);
  }
  // Send request to server
  if (compute_effector_camera_quick_client_.call(srv_chess2base)) {
    ROS_INFO_STREAM("hand_camera: "<< std::endl << srv_chess2base.response.effector_camera);
    geometry_msgs::TransformStamped chess2base_tf;
    res.success = true;
    res.message = "Calibration successful";
    chess2base_tf.transform = srv_chess2base.response.effector_camera;
    Eigen::Affine3d chess2base_eig = tf2::transformToEigen(chess2base_tf);
    geometry_msgs::TransformStamped base2chess_tf =
      tf2::eigenToTransform(chess2base_eig.inverse());
    base2chess_tf.header.frame_id = "base_link";
    base2chess_tf.child_frame_id = "chess_link";
    base2chess_tf.header.stamp = ros::Time::now();
    res.base2chess = base2chess_tf;
    ROS_INFO_STREAM(res);
  }
  else {
    res.success = false;
    res.message = "Failed to call service";
    ROS_ERROR("Failed to call service");
    success &= false;
  }
  return success;
}
