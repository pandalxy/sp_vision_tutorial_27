#include "camera.hpp"

#include <stdexcept>
#include <unordered_map>

#include "hikrobot/include/MvCameraControl.h"

// 将相机输出的 Bayer 格式原始数据转换为 BGR 彩色图像（来自 example.cpp）
cv::Mat transfer(MV_FRAME_OUT & raw)
{
  MV_CC_PIXEL_CONVERT_PARAM cvt_param;
  cv::Mat img(cv::Size(raw.stFrameInfo.nWidth, raw.stFrameInfo.nHeight), CV_8U, raw.pBufAddr);

  cvt_param.nWidth = raw.stFrameInfo.nWidth;
  cvt_param.nHeight = raw.stFrameInfo.nHeight;

  cvt_param.pSrcData = raw.pBufAddr;
  cvt_param.nSrcDataLen = raw.stFrameInfo.nFrameLen;
  cvt_param.enSrcPixelType = raw.stFrameInfo.enPixelType;

  cvt_param.pDstBuffer = img.data;
  cvt_param.nDstBufferSize = img.total() * img.elemSize();
  cvt_param.enDstPixelType = PixelType_Gvsp_BGR8_Packed;

  auto pixel_type = raw.stFrameInfo.enPixelType;
  const static std::unordered_map<MvGvspPixelType, cv::ColorConversionCodes> type_map = {
    {PixelType_Gvsp_BayerGR8, cv::COLOR_BayerGR2RGB},
    {PixelType_Gvsp_BayerRG8, cv::COLOR_BayerRG2RGB},
    {PixelType_Gvsp_BayerGB8, cv::COLOR_BayerGB2RGB},
    {PixelType_Gvsp_BayerBG8, cv::COLOR_BayerBG2RGB}};
  cv::cvtColor(img, img, type_map.at(pixel_type));

  return img;
}

// 构造函数：打开相机并开始采集
Camera::Camera()
: handle_(nullptr)
{
  // 枚举当前连接的 USB 相机
  MV_CC_DEVICE_INFO_LIST device_list;
  auto ret = MV_CC_EnumDevices(MV_USB_DEVICE, &device_list);
  if (ret != MV_OK) {
    throw std::runtime_error("枚举相机失败!");
  }
  if (device_list.nDeviceNum == 0) {
    throw std::runtime_error("没有检测到相机，请检查相机是否连接!");
  }

  // 为第一个相机创建句柄并打开
  ret = MV_CC_CreateHandle(&handle_, device_list.pDeviceInfo[0]);
  if (ret != MV_OK) {
    throw std::runtime_error("创建相机句柄失败!");
  }

  ret = MV_CC_OpenDevice(handle_);
  if (ret != MV_OK) {
    throw std::runtime_error("打开相机失败!");
  }

  // 设置相机参数
  MV_CC_SetEnumValue(handle_, "BalanceWhiteAuto", MV_BALANCEWHITE_AUTO_CONTINUOUS);
  MV_CC_SetEnumValue(handle_, "ExposureAuto", MV_EXPOSURE_AUTO_MODE_OFF);
  MV_CC_SetEnumValue(handle_, "GainAuto", MV_GAIN_MODE_OFF);
  MV_CC_SetFloatValue(handle_, "ExposureTime", 3000);  // 曝光时间, 单位us, 越大画面越亮
  MV_CC_SetFloatValue(handle_, "Gain", 5);             // 增益, 单位dB, 越大画面越亮
  MV_CC_SetFrameRate(handle_, 60);

  // 开始采集图像
  ret = MV_CC_StartGrabbing(handle_);
  if (ret != MV_OK) {
    throw std::runtime_error("开始采集失败!");
  }
}

// 读取一帧图像
cv::Mat Camera::read()
{
  MV_FRAME_OUT raw;
  unsigned int nMsec = 100;

  auto ret = MV_CC_GetImageBuffer(handle_, &raw, nMsec);
  if (ret != MV_OK) {
    return {};  // 读取失败时返回空图像
  }

  cv::Mat img = transfer(raw);

  MV_CC_FreeImageBuffer(handle_, &raw);

  return img;
}

// 析构函数：停止采集并关闭相机
Camera::~Camera()
{
  MV_CC_StopGrabbing(handle_);
  MV_CC_CloseDevice(handle_);
  MV_CC_DestroyHandle(handle_);
}
