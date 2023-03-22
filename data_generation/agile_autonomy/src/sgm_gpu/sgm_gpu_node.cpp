/***********************************************************************
  Copyright (C) 2020 Hironori Fujimoto

  This program is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.
 
  This program is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.
  You should have received a copy of the GNU General Public License
  along with this program.  If not, see <http://www.gnu.org/licenses/>.
***********************************************************************/

#include "sgm_gpu/sgm_gpu_node.h"

namespace sgm_gpu
{

SgmGpuNode::SgmGpuNode()
{
  node_handle_.reset(new ros::NodeHandle());
  private_node_handle_.reset(new ros::NodeHandle("~"));

  image_transport_.reset(new image_transport::ImageTransport(*node_handle_));

  sgm_.reset(new SgmGpu(*private_node_handle_));

  disparity_pub_ = private_node_handle_->advertise<sensor_msgs::Image>("disparity", 1);
  // Subscribe left and right Image topic
  std::string left_base_topic = node_handle_->resolveName("rgb/left");
  std::string right_base_topic = node_handle_->resolveName("rgb/right");
  left_image_sub_.subscribe(*image_transport_, left_base_topic, 10);
  right_image_sub_.subscribe(*image_transport_, right_base_topic, 10);

  // Find CameraInfo topic from corresponded Image topic and subscribe it
  // std::string left_info_topic = image_transport::getCameraInfoTopic(left_base_topic);
  // std::string right_info_topic = image_transport::getCameraInfoTopic(right_base_topic);
  // left_info_sub_.subscribe(*node_handle_, left_info_topic, 10);
  // right_info_sub_.subscribe(*node_handle_, right_info_topic, 10);

  stereo_synchronizer_.reset(
    new StereoSynchronizer(left_image_sub_, right_image_sub_, 10)
  );
  stereo_synchronizer_->registerCallback(&SgmGpuNode::stereoCallback, this);
}

void SgmGpuNode::stereoCallback(
  const sensor_msgs::ImageConstPtr &left_image,
  const sensor_msgs::ImageConstPtr &right_image)
{
  cv::Mat left = cv_bridge::toCvShare(left_image, "bgr8")->image;
  cv::Mat right = cv_bridge::toCvShare(right_image, "bgr8")->image;
  cv::Mat disp = cv::Mat(left.rows, left.cols, CV_8UC1);
  // std::cout<<left.size<<std::endl;
  // cv::imshow("lalala", left);
  // cv::imshow("lblblb", right);
  // std::cout<<"lalala"<<std::endl;
  sgm_->computeDisparity(left, right, &disp);
  int img_rows_ = left.rows;
  int img_cols_ = left.cols;
  cv::Mat disparity;
  disp.copyTo(disparity);
  disparity.convertTo(disparity, CV_32FC1);

  // compute depth from disparity
  cv::Mat depth_float(img_rows_, img_cols_, CV_32FC1);
  cv::Mat depth = cv::Mat(img_rows_, img_cols_, CV_16UC1);
  float f = 320;
  //  depth = static_cast<float>(stereo_baseline_) * f / disparity;
  for (int r = 0; r < img_rows_; ++r) {
    for (int c = 0; c < img_cols_; ++c) {
      if (disparity.at<float>(r, c) == 0.0f) {
        depth_float.at<float>(r, c) = 0.0f;
        depth.at<unsigned short>(r, c) = 0;
      } else if (disparity.at<float>(r, c) == 255.0f) {
        depth_float.at<float>(r, c) = 0.0f;
        depth.at<unsigned short>(r, c) = UINT16_MAX;
      } else {
        depth_float.at<float>(r, c) = static_cast<float>(0.16) * f /
                                      disparity.at<float>(r, c);
        depth.at<unsigned short>(r, c) = static_cast<unsigned short>(
            1000.0 * static_cast<float>(0.16) * f /
            disparity.at<float>(r, c));
      }
    }
  }

  // cv::imshow("lclclc", disparity);
  // cv::waitKey(0);
  sensor_msgs::ImagePtr disparity_msg =
      cv_bridge::CvImage(std_msgs::Header(), "mono16", depth)
          .toImageMsg();
  disparity_msg->header = left_image->header;
  disparity_pub_.publish(disparity_msg);
}

} // namespace sgm_gpu
