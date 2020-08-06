//
// Created by xcy on 2020/8/5.
//

#include "zapriltag_ros/apriltag_ros_wrapper.h"

#include <utility>
ROSWrapper::ROSWrapper(std::string tag_family_name,double tag_size,const double* intrinsic_parameter,const float* detector_parameter,bool refine_edges)
{
    aprilTag = new AprilTag(std::move(tag_family_name),tag_size,intrinsic_parameter,detector_parameter,refine_edges);
    this->image_received= false;
    it_ = new image_transport::ImageTransport(n_);

    tag_pub_ = n_.advertise<zapriltag_ros::TagsDetection_msg>("TagsDetected",100);
    image_sub_ = it_->subscribe("/camera/color/image_raw", 100,&ROSWrapper::ImageCallback,this);
    //camera_info_sub_ = n_->subscribe("/camera/camera_info",100,&ROSWrapper::InfoCallback,this);
    watchdog_timer_ = n_.createTimer(ros::Duration(WATCHDOG_PERIOD_), &ROSWrapper::watchdog, this, true);
}
ROSWrapper::~ROSWrapper()
{
    delete it_;
}
void ROSWrapper::ImageCallback(const sensor_msgs::ImageConstPtr &msg)
{
    watchdog_timer_.stop();
    watchdog_timer_.start();
    this->image_received=true;
    try
    {
        cv_ptr_ = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }
    subscribed_rgb_=cv_ptr_->image;
    n_.param<bool>("/zapriltag_ros/tagDetectorOn",tagDetectorOn_,false);
    n_.param<bool>("/zapriltag_ros/tagGraphOn",tagGraphOn_,false);
    n_.param<bool>("/zapriltag_ros/colorOn",colorOn_,false);
    if(tagDetectorOn_)
    {
        TagsInfoPublish(aprilTag->GetTargetPoseMatrix(subscribed_rgb_));
        if(tagGraphOn_)
        {
            cv::imshow("Tag Detections", subscribed_rgb_);
            cv::waitKey(3);
        }
    }
    else
    {
        if(colorOn_)
        {
            cv::imshow("Tag Detections",subscribed_rgb_);
            cv::waitKey(3);
        }

    }

}

void ROSWrapper::TagsInfoPublish(const tag_detection_info_t& tags_detected)
{
    zapriltag_ros::TagsDetection_msg TagsDetection;
    zapriltag_ros::TagDetection_msg TagDetection;
    if(!tags_detected.empty())
    {
        //std::cout<<tags_detected_[0].Trans_C2T.matrix()<<std::endl;

        TagsDetection.header.frame_id="camera_color_optical_frame";
        for(int i=0;i<tags_detected.size();++i)
        {
            TagDetection.id = tags_detected[i].id;
            TagDetection.pose = tf2::toMsg(tags_detected[i].Trans_C2T);
            TagDetection.PixelCoef = tags_detected[i].PixelCoef;
            TagDetection.center.x = tags_detected[i].Center.x;
            TagDetection.center.y = tags_detected[i].Center.y;
            TagsDetection.tags_information.push_back(TagDetection);
        }
    }
    tag_pub_.publish(TagsDetection);
}
void ROSWrapper::watchdog(const ros::TimerEvent &e)
{
    ROS_WARN("Image not received for %f seconds, is the camera node drop?", WATCHDOG_PERIOD_);
    this->image_received=false;
}