//
// Created by xcy on 2020/8/5.
//

#ifndef ZAPRILTAG_H
#define ZAPRILTAG_H

#include <cstdint>
#include <unistd.h>
#include <cstdio>
#include <cstdlib>
#include <iostream>
#include <fstream>
#include <sstream>
#include <algorithm>
#include <string>
#include <cmath>
#include <ctime>
#include <utility>

#include "Eigen/Dense"
#include "Eigen/Geometry"
#include <Eigen/Core>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/utility.hpp>
#include <opencv2/ximgproc.hpp>
#include <opencv2/core/eigen.hpp>
#include "cv.h"
#include "highgui.h"

#include <boost/thread/thread.hpp>
#include <boost/thread/locks.hpp>
#include <boost/thread/shared_mutex.hpp>


#include "tr1/memory"

#include <dirent.h>

extern "C" {
#include "apriltag.h"
#include "tag36h11.h"
#include "tag25h9.h"
#include "tag16h5.h"
#include "tagCircle21h7.h"
#include "tagCircle49h12.h"
#include "tagCustom48h12.h"
#include "tagStandard41h12.h"
#include "tagStandard52h13.h"
#include "common/getopt.h"
#include "apriltag_pose.h"
}

struct TagDetectInfo{
    //the tag pose with respect to camera described as a homogeneous matrix
    Eigen::Affine3d Trans_C2T;
    //the tag id
    int id;
    //the coefficient unit meters per pixel
    double PixelCoef;
    //the pixel point of tag center
    cv::Point2d Center;
};

typedef std::vector<TagDetectInfo> tag_detection_info_t;

class AprilTag{
private:
    TagDetectInfo TagDetected_;

    //tag family name
    const std::string TAG_FAMILY_NAME_;
    //april tag handle
    apriltag_family_t *tf;
    //apriltag detecter handle
    apriltag_detector_t *td;
    //apriltag detection handle
    apriltag_detection_t *det;
    //apriltag detection info handle;
    apriltag_detection_info_t info;
    //apriltag pose handle
    apriltag_pose_t pose;

    //april tag physic size unit meter
    const double TAG_SIZE_ ;



public:
    bool image_received;
    AprilTag(std::string tag_family_name,double tag_size,const double* intrinsic_parameter,
             const float* detector_parameter,bool refine_edges);
    ~AprilTag();

    /*
     * Function computes tag information of all tags detected
     * @param UserImage [the image prepared to detect tag]
     * fill a vector contains all tag information detected, empty when no tags detected
     */
    tag_detection_info_t GetTargetPoseMatrix(cv::Mat & UserImage);

};

#endif //ZAPRILTAG_H