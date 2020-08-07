//
// Created by xcy on 2020/8/5.
//

#ifndef ZAPRILTAG_ROS_APRILTAG_ROS_WRAPPER_H
#define ZAPRILTAG_ROS_APRILTAG_ROS_WRAPPER_H

#include "zapriltag_ros/zapriltag.h"

#include <ros/ros.h>
#include <ros/forwards.h>
#include <ros/single_subscriber_publisher.h>
#include <sensor_msgs/Image.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf2_eigen/tf2_eigen.h>
#include "ros/callback_queue.h"
#include "tf/transform_datatypes.h"
#include "tf/transform_broadcaster.h"

#include "zapriltag_ros/TagDetection_msg.h"
#include "zapriltag_ros/TagsDetection_msg.h"

class ROSWrapper {
private:
    AprilTag *aprilTag;

    // watchdog period
    const double WATCHDOG_PERIOD_ = 1.0;
    //triggers
    bool tagGraphOn_{},colorOn_{},tagDetectorOn_{};
    bool publish_tf_{};
    ros::NodeHandle n_;
    image_transport::ImageTransport *it_;
    image_transport::Subscriber image_sub_;

    ros::Subscriber camera_info_sub_;
    ros::Publisher tag_pub_;

    tf::TransformBroadcaster broad_caster;

    cv_bridge::CvImagePtr cv_ptr_;
    cv::Mat subscribed_rgb_;
    ros::Timer watchdog_timer_;

    void watchdog(const ros::TimerEvent &e);
    void ImageCallback(const sensor_msgs::ImageConstPtr& msg);
    void TagsInfoPublish(const tag_detection_info_t & tags_detected,const std::string & frame_id);
    void TFPublish(const tag_detection_info_t & tags_detected,const std::string & frame_id);
public:
    bool image_received{};
    ROSWrapper(std::string tag_family_name,double tag_size,const double* intrinsic_parameter,
               const float* detector_parameter,bool refine_edges);
    ~ROSWrapper();
};


#endif //ZAPRILTAG_ROS_APRILTAG_ROS_WRAPPER_H
