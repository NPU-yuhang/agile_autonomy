#ifndef SGM_GPU__SGM_GPU_NODE_H_
#define SGM_GPU__SGM_GPU_NODE_H_

#include "sgm_gpu/sgm_gpu.h"

#include <image_transport/camera_common.h>
#include <image_transport/image_transport.h>
#include <image_transport/subscriber_filter.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

namespace sgm_gpu
{

class SgmGpuNode
{
private:
  std::shared_ptr<ros::NodeHandle> node_handle_;
  std::shared_ptr<ros::NodeHandle> private_node_handle_;

  std::shared_ptr<image_transport::ImageTransport> image_transport_;

  std::shared_ptr<SgmGpu> sgm_;

  image_transport::SubscriberFilter left_image_sub_;
  image_transport::SubscriberFilter right_image_sub_;
  message_filters::Subscriber<sensor_msgs::CameraInfo> left_info_sub_;
  message_filters::Subscriber<sensor_msgs::CameraInfo> right_info_sub_;

  using StereoSynchronizer = message_filters::TimeSynchronizer
  <sensor_msgs::Image, sensor_msgs::Image>;
  std::shared_ptr<StereoSynchronizer> stereo_synchronizer_;

  ros::Publisher disparity_pub_;
  
  void stereoCallback(
    const sensor_msgs::ImageConstPtr &left_image_msg, 
    const sensor_msgs::ImageConstPtr &right_image_msg);

public:
  SgmGpuNode();
};

} // namespace sgm_gpu

#endif
