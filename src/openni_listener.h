#ifndef OPENNI_LISTENER_H 
#define OPENNI_LISTENER_H 

#include "ros/ros.h"                                        
#include <message_filters/subscriber.h>                     
#include <message_filters/synchronizer.h>                   
#include <message_filters/sync_policies/approximate_time.h> 
#include <sensor_msgs/Image.h>                              
#include <sensor_msgs/CameraInfo.h>                         
                                                            
#include <rosbag/bag.h>                                     
//For rosbag reading                                        
#include <rosbag/view.h>                                    
#include <boost/foreach.hpp>                                


template <class M>
class BagSubscriber : public message_filters::SimpleFilter<M>         
{   
  public:
  void newMessage(const boost::shared_ptr<M const> &msg) 
  {
    this->signalMessage(msg); //"this->" is required as of ros groovy
  } 
};  


typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image,                         
                                                        sensor_msgs::Image,
                                                        sensor_msgs::CameraInfo> NoCloudSyncPolicy;


class OpenNIListener
{
  public:
    OpenNIListener();
    void loadBagFile(std::string filename);   
    void loadBagFakeSubscriberSetup(const std::string& visua_tpc,
                                    const std::string& depth_tpc,
                                    const std::string& cinfo_tpc);
    
    void noCloudCallback (const sensor_msgs::ImageConstPtr& visual_img_msg,   
                          const sensor_msgs::ImageConstPtr& depth_img_msg,    
                          const sensor_msgs::CameraInfoConstPtr& cam_info_msg);
    
    void processBagfile(std::string filename,
                        const std::string& visua_tpc,
                        const std::string& depth_tpc,
                        const std::string& cinfo_tpc);

    message_filters::Synchronizer<NoCloudSyncPolicy>* no_cloud_sync_;
    BagSubscriber<sensor_msgs::Image>* rgb_img_sub_;      
    BagSubscriber<sensor_msgs::Image>* depth_img_sub_;
    BagSubscriber<sensor_msgs::CameraInfo>* cam_info_sub_;
    int count;
};

#endif
