#pragma once
#include <ros/ros.h>
#include <opencv2/opencv.hpp>

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
