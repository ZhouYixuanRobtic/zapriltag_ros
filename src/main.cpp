
#include "zapriltag_ros/apriltag_ros_wrapper.h"
int main(int argc, char* argv[])
{
    ros::init(argc,argv,"zapriltag_ros");
    ros::NodeHandle nh_("~");
    double intrinsic_parameter[4]{};
    float detector_parameter[3]{};
    double tag_size;std::string tag_family_name;
    bool refine_edges{true};
    nh_.param<double>("fx",intrinsic_parameter[0],615.3072);
    nh_.param<double>("fy",intrinsic_parameter[1],616.1456);
    nh_.param<double>("u0",intrinsic_parameter[2],333.4404);
    nh_.param<double>("v0",intrinsic_parameter[3],236.2650);
    nh_.param<double>("tag_size",tag_size,0.1);
    nh_.param<std::string>("tag_family_name",tag_family_name,"tag36h11");
    nh_.param<float>("quad_decimate",detector_parameter[0],2.0);
    nh_.param<float>("quad_sigma",detector_parameter[1],0.0);
    nh_.param<float>("nthreads",detector_parameter[2],2.0);
    nh_.param<bool>("refine_edges",refine_edges,true);

    ROSWrapper real_sense(tag_family_name,tag_size,intrinsic_parameter,detector_parameter,refine_edges);
    ros::Rate loop_rate(30);
    while(ros::ok())
    {
       ros::spinOnce();
       loop_rate.sleep();
    }
   return 0;
}

