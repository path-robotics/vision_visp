#include <ros/ros.h>
#include "visp_hand2eye_calibration/glugun_client.h"

int main(int argc,char**argv){
  ros::init(argc, argv, "visp_hand2eye_calibration/glugun_client");

  glugun::Client client;
  ros::spin();
  return 0 ;
}
