#include "openni_listener.h"   

int main(int argc, char **argv)
{
  ros::init(argc, argv, "listener");

  ros::NodeHandle nh;

  OpenNIListener listener;
  
  ros::spin();

  return 0;
}
