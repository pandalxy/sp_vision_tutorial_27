#ifndef IO__CAMERA_HPP
#define IO__CAMERA_HPP

#include <opencv2/opencv.hpp>

// 相机类：封装了 example.cpp 中打开相机、读取一帧图像、关闭相机的流程
class Camera
{
public:
  Camera();
  ~Camera();

  cv::Mat read();  // 读取一帧图像

private:
  void * handle_;  // 相机句柄
};

#endif  // IO__CAMERA_HPP
