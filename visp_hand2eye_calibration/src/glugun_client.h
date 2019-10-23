#include <ros/ros.h>
#include <glugun_camera/LocateCamera.h>
#include "visp_hand2eye_calibration/compute_effector_camera_quick.h"

#ifndef __GLUGUN_CLIENT_H_INCLUDED__
#define __GLUGUN_CLIENT_H_INCLUDED__

namespace glugun {

class Client {
 public:
  ros::NodeHandle nh_;
  ros::ServiceClient compute_effector_camera_quick_client_;

  // LocateCamera Server
  ros::ServiceServer locate_camera_server_;
  // Server callback function
  bool locate_camera_cb(glugun_camera::LocateCamera::Request &req,
                        glugun_camera::LocateCamera::Response &res);

 public:
  Client();
  ~Client();
};

}

#endif
