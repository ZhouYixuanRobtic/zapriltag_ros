//
// Created by xcy on 2020/8/5.
//

#include "zapriltag_ros/zapriltag.h"

AprilTag::AprilTag(std::string tag_family_name,double tag_size,const double* intrinsic_parameter,const float* detector_parameter,bool refine_edges):
TAG_FAMILY_NAME_(std::move(tag_family_name)),TAG_SIZE_(tag_size)

{
    //camera intrinsic parameter
    info.fx = intrinsic_parameter[0];
    info.fy = intrinsic_parameter[1];
    info.cx = intrinsic_parameter[2];
    info.cy = intrinsic_parameter[3];
    info.tagsize = tag_size;

    if (TAG_FAMILY_NAME_ == "tag36h11")
    {
        tf = tag36h11_create();
    }
    else if (TAG_FAMILY_NAME_ == "tag25h9")
    {
        tf = tag25h9_create();
    }
    else if (TAG_FAMILY_NAME_ == "tag16h5")
    {
        tf = tag16h5_create();
    }
    else if (TAG_FAMILY_NAME_ == "tagCircle21h7")
    {
        tf = tagCircle21h7_create();
    }
    else if (TAG_FAMILY_NAME_ == "tagCircle49h12")
    {
        tf = tagCircle49h12_create();
    } else if (TAG_FAMILY_NAME_ == "tagStandard41h12")
    {
        tf = tagStandard41h12_create();
    } else if (TAG_FAMILY_NAME_ == "tagStandard52h13")
    {
        tf = tagStandard52h13_create();
    } else if (TAG_FAMILY_NAME_ == "tagCustom48h12")
    {
        tf = tagCustom48h12_create();
    }
    else {
        printf("Unrecognized tag family name. Use e.g. \"tag36h11\".\n");
        exit(-1);
    }

    td = apriltag_detector_create();
    td->quad_decimate = detector_parameter[0];
    td->quad_sigma = detector_parameter[1];
    td->nthreads = (int)detector_parameter[2];
    td->debug = false;
    td->refine_edges = refine_edges;

    apriltag_detector_add_family(td, tf);
}
AprilTag::~AprilTag()
{
    apriltag_detector_destroy(td);

    if (TAG_FAMILY_NAME_ == "tag36h11")
    {
        tag36h11_destroy(tf);
    }
    else if (TAG_FAMILY_NAME_ == "tag25h9")
    {
        tag25h9_destroy(tf);
    }
    else if (TAG_FAMILY_NAME_ == "tag16h5")
    {
        tag16h5_destroy(tf);
    }
    else if (TAG_FAMILY_NAME_ == "tagCircle21h7")
    {
        tagCircle21h7_destroy(tf);
    }
    else if (TAG_FAMILY_NAME_ == "tagCircle49h12")
    {
        tagCircle49h12_destroy(tf);
    } else if (TAG_FAMILY_NAME_ == "tagStandard41h12")
    {
        tagStandard41h12_destroy(tf);
    } else if (TAG_FAMILY_NAME_ == "tagStandard52h13")
    {
        tagStandard52h13_destroy(tf);
    } else
    {
        tagCustom48h12_destroy(tf);
    }

}

tag_detection_info_t AprilTag::GetTargetPoseMatrix(cv::Mat & UserImage)
{
    cv::Mat gray;
    //Tags information detected from camera, vector is empty when no tags are detected
    tag_detection_info_t tags_detected;
    cvtColor(UserImage, gray, cv::COLOR_BGR2GRAY);
    // Make an image_u8_t header for the Mat data
    image_u8_t im = { .width = gray.cols,
            .height = gray.rows,
            .stride = gray.cols,
            .buf = gray.data
    };
    zarray_t *detections = apriltag_detector_detect(td, &im);
    for (int i = 0; i < zarray_size(detections); i++)
    {
        zarray_get(detections, i, &det);
        line(UserImage, cv::Point(det->p[0][0], det->p[0][1]),
             cv::Point(det->p[1][0], det->p[1][1]),
             cv::Scalar(0, 0xff, 0), 2);
        line(UserImage, cv::Point(det->p[0][0], det->p[0][1]),
             cv::Point(det->p[3][0], det->p[3][1]),
             cv::Scalar(0, 0, 0xff), 2);
        line(UserImage, cv::Point(det->p[1][0], det->p[1][1]),
             cv::Point(det->p[2][0], det->p[2][1]),
             cv::Scalar(0xff, 0, 0), 2);
        line(UserImage, cv::Point(det->p[2][0], det->p[2][1]),
             cv::Point(det->p[3][0], det->p[3][1]),
             cv::Scalar(0xff, 0, 0), 2);

        std::stringstream ss;
        ss << det->id;
        cv::String text = ss.str();
        int baseline;
        cv::Size textsize = getTextSize(text, cv::FONT_HERSHEY_SCRIPT_SIMPLEX, 1.0, 2,
                                    &baseline);
        putText(UserImage, text, cv::Point(det->c[0]-(float)textsize.width/2,
                                       det->c[1]+(float)textsize.height/2),
                cv::FONT_HERSHEY_SCRIPT_SIMPLEX, 1.0, cv::Scalar(0xff, 0x99, 0), 2);
        //pose_estimation
        info.det = det;
        double err = estimate_tag_pose(&info, &pose);
        TagDetected_.Trans_C2T.matrix() << pose.R->data[0],  pose.R->data[1],  pose.R->data[2],  pose.t->data[0],
                pose.R->data[3],  pose.R->data[4],  pose.R->data[5],  pose.t->data[1],
                pose.R->data[6],  pose.R->data[7],  pose.R->data[8],  pose.t->data[2],
                0.0,  0.0,  0.0,  1.0;
        TagDetected_.id=det->id;
        TagDetected_.Center=cv::Point2d(det->c[0],det->c[1]);
        TagDetected_.PixelCoef=0.0;
        TagDetected_.PixelCoef+=TAG_SIZE_/norm(cv::Point(det->p[0][0]-det->p[1][0],det->p[0][1]-det->p[1][1]));
        TagDetected_.PixelCoef+=TAG_SIZE_/norm(cv::Point(det->p[0][0]-det->p[3][0],det->p[0][1]-det->p[3][1]));
        TagDetected_.PixelCoef+=TAG_SIZE_/norm(cv::Point(det->p[1][0]-det->p[2][0],det->p[1][1]-det->p[2][1]));
        TagDetected_.PixelCoef+=TAG_SIZE_/norm(cv::Point(det->p[2][0]-det->p[3][0],det->p[2][1]-det->p[3][1]));
        TagDetected_.PixelCoef+=sqrt(2)*TAG_SIZE_/norm(cv::Point(det->p[0][0]-det->p[2][0],det->p[0][1]-det->p[2][1]));
        TagDetected_.PixelCoef+=sqrt(2)*TAG_SIZE_/norm(cv::Point(det->p[1][0]-det->p[3][0],det->p[1][1]-det->p[3][1]));
        TagDetected_.PixelCoef=TagDetected_.PixelCoef/6;
        tags_detected.push_back(TagDetected_);
    }
    zarray_destroy(detections);
    return tags_detected;
}


