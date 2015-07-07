#ifndef UTILS_H
#define UTILs_H

#include <cmath>
#include <cstdio>

//ROS
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>

//OpenCV
#include <opencv2/opencv.hpp>

//Eigen
#include <Eigen/Eigen>

cv::Mat disparity(int alg, cv::Mat framel, cv::Mat framer);

cv::Mat coloredDisparity(cv::Mat disp8);

cv::Mat rectifiedPair(cv::Mat framel, cv::Mat framer);

void fromQuaternionToRotationMatrix(double qx,double qy,double qz, double qw, Eigen::Matrix3d &rot);

bool loadIntrinsicFromYAML(const char *filename, sensor_msgs::CameraInfo &info_cam, cv::Mat& M, cv::Mat& D, cv::Mat& R, cv::Mat& P);

bool loadIntrinsicFromXML(const char *filename, sensor_msgs::CameraInfo &info_cam, cv::Mat& M, cv::Mat& D);

bool loadExtrinsicFromXML(const char *filename, sensor_msgs::CameraInfo &info_caml, sensor_msgs::CameraInfo &info_camr, cv::Mat& R1, cv::Mat& P1, cv::Mat& R2, cv::Mat& P2);

cv::Mat makeQMatrix(cv::Point2d image_center,double focal_length, double baseline);

void fillOcclusion(cv::Mat& src, int invalidvalue);

template <class T>
static void fillOcclusion_(cv::Mat& src, const T invalidvalue, const T maxval);

#endif
