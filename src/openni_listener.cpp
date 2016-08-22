#include"openni_listener.h"

OpenNIListener::OpenNIListener()
{
  count=0;
  //ROS_INFO("构造函数");
  std::string bagfile_name ="/home/leone/Documents/bechmark/rgbd_dataset_freiburg1_desk.bag";
  loadBagFile(bagfile_name);
}
void OpenNIListener::loadBagFile(std::string filename)
{  
       
  std::string visua_tpc  = "/camera/rgb/image_color";                                                           
  std::string depth_tpc  = "/camera/depth/image";
  std::string cinfo_tpc  = "/camera/rgb/camera_info";
  //std::string tf_tpc     = std::string("/tf");   
  loadBagFakeSubscriberSetup(visua_tpc, depth_tpc, cinfo_tpc);
  processBagfile(filename, visua_tpc, depth_tpc, cinfo_tpc);
                             
}
void OpenNIListener::loadBagFakeSubscriberSetup(const std::string& visua_tpc,
                                                const std::string& depth_tpc,
                                                const std::string& cinfo_tpc)
{
  //ROS_INFO("SUBSCRIBERSETUP"); 
  //All information from Kinect
  if(!visua_tpc.empty() && !depth_tpc.empty() && !cinfo_tpc.empty())
  {   
    // Set up fake subscribers to capture images
    depth_img_sub_ = new BagSubscriber<sensor_msgs::Image>();
    rgb_img_sub_ = new BagSubscriber<sensor_msgs::Image>();
    cam_info_sub_ = new BagSubscriber<sensor_msgs::CameraInfo>();
    no_cloud_sync_ = new message_filters::Synchronizer<NoCloudSyncPolicy>(NoCloudSyncPolicy(3),  *rgb_img_sub_, *depth_img_sub_,*cam_info_sub_);
    no_cloud_sync_->registerCallback(boost::bind(&OpenNIListener::noCloudCallback, this, _1, _2, _3));
    ROS_INFO_STREAM_NAMED("OpenNIListener", "Listening to " << visua_tpc << ", " << depth_tpc << " and " << cinfo_tpc);
  } 
    
}

bool asyncFrameDrop(ros::Time depth, ros::Time rgb)                                                   
{            
  long rgb_timediff = abs(static_cast<long>(rgb.nsec) - static_cast<long>(depth.nsec));
  if(rgb_timediff > 33333333)
  {
    ROS_DEBUG("Depth image time: %d - %d", depth.sec,   depth.nsec);
    ROS_DEBUG("RGB   image time: %d - %d", rgb.sec, rgb.nsec);
    ROS_INFO("Depth and RGB image off more than 1/30sec: %li (nsec)", rgb_timediff);
    return true;
           
  } 
  else 
  {   
    ROS_DEBUG("Depth image time: %d - %d", depth.sec,   depth.nsec);
    ROS_DEBUG("RGB   image time: %d - %d", rgb.sec, rgb.nsec);
  }          
  return false;
}            


void OpenNIListener::noCloudCallback (const sensor_msgs::ImageConstPtr& visual_img_msg,
                                      const sensor_msgs::ImageConstPtr& depth_img_msg,    
                                      const sensor_msgs::CameraInfoConstPtr& cam_info_msg)
{
  if(asyncFrameDrop(depth_img_msg->header.stamp, visual_img_msg->header.stamp)) 
    return;   

  //count++;
  //std::cout<<"image:"<<count<<std::endl;
  //ROS_INFO("CALLBACK");
  //sleep(1);
}

void OpenNIListener::processBagfile(std::string filename,
                                    const std::string& visua_tpc,
                                    const std::string& depth_tpc,
                                    const std::string& cinfo_tpc)
                                    
{
 
  //ROS_INFO("PROCESSING");
  ROS_INFO_NAMED("OpenNIListener", "Loading Bagfile %s", filename.c_str());
  { //bag will be destructed after this block (hopefully frees memory for the optimizer)
    rosbag::Bag input_bag;
    try{
      input_bag.open(filename, rosbag::bagmode::Read);
    } catch(rosbag::BagIOException ex) {
      ROS_FATAL("Opening Bagfile %s failed: %s Quitting!", filename.c_str(), ex.what());
      ros::shutdown();
      return;
    }
    ROS_INFO_NAMED("OpenNIListener", "Opened Bagfile %s", filename.c_str());

    std::vector<std::string> topics;
    topics.push_back(visua_tpc);
    topics.push_back(depth_tpc);
    topics.push_back(cinfo_tpc);
    //topics.push_back(tf_tpc);

    rosbag::View view(input_bag, rosbag::TopicQuery(topics));
    // Simulate sending of the messages in the bagfile
    std::deque<sensor_msgs::Image::ConstPtr> vis_images;
    std::deque<sensor_msgs::Image::ConstPtr> dep_images;
    std::deque<sensor_msgs::CameraInfo::ConstPtr> cam_infos;
    
    ros::Time last_tf=ros::TIME_MIN;

    BOOST_FOREACH(rosbag::MessageInstance const m, view)
    {
     /* do{ 
        usleep(10);
        if(!ros::ok()) return;
      } while(pause_);
     */
      ROS_INFO_NAMED("OpenNIListener", "Processing %s of type %s with timestamp %f", m.getTopic().c_str(), m.getDataType().c_str(), m.getTime().toSec());

      if (m.getTopic() == visua_tpc || ("/" + m.getTopic() == visua_tpc))
      {
        sensor_msgs::Image::ConstPtr rgb_img = m.instantiate<sensor_msgs::Image>();
        if (rgb_img) vis_images.push_back(rgb_img);
        ROS_DEBUG("Found Message of %s", visua_tpc.c_str());
      }
      else if (m.getTopic() == depth_tpc || ("/" + m.getTopic() == depth_tpc))
      {
        sensor_msgs::Image::ConstPtr depth_img = m.instantiate<sensor_msgs::Image>();
        //if (depth_img) depth_img_sub_->newMessage(depth_img);
        if (depth_img) dep_images.push_back(depth_img);
        ROS_DEBUG("Found Message of %s", depth_tpc.c_str());
      }
      else if (m.getTopic() == cinfo_tpc || ("/" + m.getTopic() == cinfo_tpc))
      {
        sensor_msgs::CameraInfo::ConstPtr cam_info = m.instantiate<sensor_msgs::CameraInfo>();
        //if (cam_info) cam_info_sub_->newMessage(cam_info);
        if (cam_info)
        {
          cam_infos.push_back(cam_info);
          
          last_tf = cam_info->header.stamp;
          last_tf -= ros::Duration(0.1);
          ROS_DEBUG("Found Message of %s", cinfo_tpc.c_str());
        }
      }  

      if (last_tf == ros::TIME_MIN)
      {
        //If not a valid time yet, set to something before first message's stamp 
        last_tf = m.getTime();
        last_tf -= ros::Duration(0.1);
      }    

      //last_tf = m.getTime();//FIXME: No TF -> no processing
      while(!vis_images.empty() && vis_images.front()->header.stamp < last_tf){
          ROS_INFO_NAMED("OpenNIListener", "Forwarding buffered visual message from time %12f", vis_images.front()->header.stamp.toSec());
          rgb_img_sub_->newMessage(vis_images.front());
          vis_images.pop_front();
      }
      while(!dep_images.empty() && dep_images.front()->header.stamp < last_tf){
          ROS_INFO_NAMED("OpenNIListener", "Forwarding buffered depth message from time %12f", dep_images.front()->header.stamp.toSec());
          depth_img_sub_->newMessage(dep_images.front());
          dep_images.pop_front();
      }
      while(!cam_infos.empty() && cam_infos.front()->header.stamp < last_tf){
          ROS_INFO_NAMED("OpenNIListener", "Forwarding buffered cam info message from time %12f", cam_infos.front()->header.stamp.toSec());
          cam_info_sub_->newMessage(cam_infos.front());
          cam_infos.pop_front();
      }

    }
    ROS_WARN_NAMED("eval", "Finished processing of Bagfile");
    input_bag.close();
  }
}


