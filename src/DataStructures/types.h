#pragma once
#include <ros/ros.h>
#include <opencv2/opencv.hpp>
#include "../util/EigenCoreInclude.h"

struct CALIBRATION_PAR{
  float fx, fy, cx, cy ;
  float d[6] ;
  int width ;
  int height ;
} ;

class ImageMeasurement
{
  public:
    ros::Time t;
    cv::Mat   image;

    ImageMeasurement(const ros::Time& _t, const cv::Mat& _image)
    {
      t     = _t;
      image = _image.clone();
    }

    ImageMeasurement(const ImageMeasurement& i)
    {
      t     = i.t;
      image = i.image.clone();
    }

    ~ImageMeasurement() { }
};

struct FRAMEINFO
{
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    Eigen::Matrix3f R_k_2_c;//R_k^(k+1)
    Eigen::Vector3f T_k_2_c;//T_k^(k+1)
    //Matrix<float, 6, 6> lastestATA ;
    Eigen::MatrixXf lastestATA ;
    bool keyFrameFlag ;
    bool trust ;
    ros::Time t ;
};

struct CAMERALINK
{
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  Eigen::Matrix3f R_bi_2_bj;//rotation from current body to link body
  Eigen::Vector3f T_bi_2_bj;//translation from current body to link body
  Eigen::Matrix<float, 6, 6> P_inv;
        //bool T_trust ;
        //MatrixXd P_inv;
};
