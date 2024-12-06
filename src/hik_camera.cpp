#include "hik_camera.hpp"

namespace hik_camera
{

  HikCamera::HikCamera()
  {
    MV_CC_DEVICE_INFO_LIST device_list;
    // enum device
    nRet = MV_CC_EnumDevices(MV_USB_DEVICE, &device_list);
    std::cout << "Enum state: " << std::hex << nRet << std::endl;
    std::cout << "Device number: " << device_list.nDeviceNum << std::endl;

    while (device_list.nDeviceNum == 0) {
      std::cout << "Not found camera, retrying..." << std::endl;
      std::cout << "Enum state: " << std::hex << nRet << std::endl;
      sleep(1);
      nRet = MV_CC_EnumDevices(MV_USB_DEVICE, &device_list);
    }

    MV_CC_CreateHandle(&camera_handle_, device_list.pDeviceInfo[0]);

    MV_CC_OpenDevice(camera_handle_);

    // Get camera infomation
    MV_CC_GetImageInfo(camera_handle_, &img_info_);

    // Init convert param
    convert_param_.nWidth = img_info_.nWidthValue;
    convert_param_.nHeight = img_info_.nHeightValue;
    convert_param_.enDstPixelType = PixelType_Gvsp_RGB8_Packed;

    declareParameters();

    MV_CC_StartGrabbing(camera_handle_);

    // Load camera info

    capture_thread_ = std::thread{[this]() -> void {
      MV_FRAME_OUT out_frame;

      

      char input;

      while (true) {
        nRet = MV_CC_GetImageBuffer(camera_handle_, &out_frame, 1000);
        if (MV_OK == nRet) {
          image_data_.resize(out_frame.stFrameInfo.nWidth * out_frame.stFrameInfo.nHeight * 3);

          convert_param_.pDstBuffer = image_data_.data();
          convert_param_.nDstBufferSize = image_data_.size();
          convert_param_.pSrcData = out_frame.pBufAddr;
          convert_param_.nSrcDataLen = out_frame.stFrameInfo.nFrameLen;
          convert_param_.enSrcPixelType = out_frame.stFrameInfo.enPixelType;

          MV_CC_ConvertPixelType(camera_handle_, &convert_param_);

          image_ = cv::Mat(out_frame.stFrameInfo.nHeight, out_frame.stFrameInfo.nWidth, CV_8UC3, image_data_.data());
          cv::cvtColor(image_, image_, cv::COLOR_RGB2BGR);
          cv::imshow("image_", image_);

          MV_CC_FreeImageBuffer(camera_handle_, &out_frame);
          fail_conut_ = 0;
        } else {
          std::cout << "Get buffer failed, retrying..." << std::endl;
          MV_CC_StopGrabbing(camera_handle_);
          MV_CC_StartGrabbing(camera_handle_);
          fail_conut_++;
        }

        if (fail_conut_ > 5) {
          std::cout << "Camera failed..." << std::endl;
        }
      }
    }};
  }

  HikCamera::~HikCamera()
  {
    if (capture_thread_.joinable()) {
      capture_thread_.join();
    }
    if (camera_handle_) {
      MV_CC_StopGrabbing(camera_handle_);
      MV_CC_CloseDevice(camera_handle_);
      MV_CC_DestroyHandle(&camera_handle_);
    }
    std::cout << "Camera closed!" << std::endl;
  }

  void HikCamera::declareParameters()
  {
    MVCC_FLOATVALUE f_value;
    // Exposure time
    MV_CC_GetFloatValue(camera_handle_, "ExposureTime", &f_value);

    int exposure_time = params_.exposure_time;
    MV_CC_SetFloatValue(camera_handle_, "ExposureTime", exposure_time);
    std::cout << "Exposure time: " << exposure_time << std::endl;

    // Gain
    MV_CC_GetFloatValue(camera_handle_, "Gain", &f_value);

    int gain = params_.gain;
    MV_CC_SetFloatValue(camera_handle_, "Gain", gain);
    std::cout << "Gain: " << gain << std::endl;
  }
}  // namespace hik_camera

int main()
{
  hik_camera::HikCamera camera;
  while (true) {
    if (cv::waitKey(1) == 'q') {
      break;
    }
  }
  return 0;
}