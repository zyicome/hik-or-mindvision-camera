#include "mindvision_camera.hpp"

namespace mindvision_camera
{

   MVCamera::MVCamera()
   {
    std::cout << "MVCameraNode created!" << std::endl;

    CameraSdkInit(1);

    // 枚举设备，并建立设备列表
    int i_camera_counts = 1;
    int i_status = -1;
    tSdkCameraDevInfo t_camera_enum_list;
    i_status = CameraEnumerateDevice(&t_camera_enum_list, &i_camera_counts);
    std::cout << "Enumerate device status = " << i_status << std::endl;
    std::cout << "Found camera count = " << i_camera_counts << std::endl;

    // 没有连接设备
    if (i_camera_counts == 0) {
      std::cout << "No camera found!" << std::endl;
      return;
    }

    // 相机初始化。初始化成功后，才能调用任何其他相机相关的操作接口
    i_status = CameraInit(&t_camera_enum_list, -1, -1, &h_camera_);

    // 初始化失败
    std::cout << "Camera init status = " << i_status << std::endl;
    if (i_status != CAMERA_STATUS_SUCCESS) {
      std::cout << "Camera init failed!" << std::endl;
      return;
    }

    // 获得相机的特性描述结构体。该结构体中包含了相机可设置的各种参数的范围信息。决定了相关函数的参数
    CameraGetCapability(h_camera_, &t_capability_);

    // 设置手动曝光
    CameraSetAeState(h_camera_, false);

    // Declare camera parameters
    declareParameters();

    // 让SDK进入工作模式，开始接收来自相机发送的图像
    // 数据。如果当前相机是触发模式，则需要接收到
    // 触发帧以后才会更新图像。
    CameraPlay(h_camera_);

    CameraSetIspOutFormat(h_camera_, CAMERA_MEDIA_TYPE_RGB8);

    capture_thread_ = std::thread{[this]() -> void {
      std::cout << "Capture thread started!" << std::endl;

      char input;

      while (true) {
        int status = CameraGetImageBuffer(h_camera_, &s_frame_info_, &pby_buffer_, 1000);
        if (status == CAMERA_STATUS_SUCCESS) {
          if (image_data_.empty() || image_data_.size() != s_frame_info_.iHeight * s_frame_info_.iWidth * 3) {
            image_data_.resize(s_frame_info_.iHeight * s_frame_info_.iWidth * 3);
          }

          CameraImageProcess(h_camera_, pby_buffer_, image_data_.data(), &s_frame_info_);
          if (flip_image_) {
            CameraFlipFrameBuffer(image_data_.data(), &s_frame_info_, 3);
          }
          
          image_ = cv::Mat(
            s_frame_info_.iHeight, s_frame_info_.iWidth, CV_8UC3, image_data_.data());
          cv::cvtColor(image_, image_, cv::COLOR_RGB2BGR);
          cv::imshow("image_", image_);
          input = cv::waitKey(1);
          if (input == 'q') {
            over = true;
            break;
          }

          // 在成功调用CameraGetImageBuffer后，必须调用CameraReleaseImageBuffer来释放获得的buffer。
          // 否则再次调用CameraGetImageBuffer时，程序将被挂起一直阻塞，
          // 直到其他线程中调用CameraReleaseImageBuffer来释放了buffer
          CameraReleaseImageBuffer(h_camera_, pby_buffer_);
          fail_conut_ = 0;
        } else {
          std::cout << "Get buffer failed, retrying..." << std::endl;
          fail_conut_++;
        }

        if (fail_conut_ > 5) {
          std::cout << "Camera failed..." << std::endl;
        }
      }
    }};
  }

  MVCamera::~MVCamera()
  {
    if (capture_thread_.joinable()) {
      capture_thread_.join();
    }

    CameraUnInit(h_camera_);

    std::cout << "Camera closed!" << std::endl;
  }

  void MVCamera::declareParameters()
  {
    // Exposure time
    // 对于CMOS传感器，其曝光的单位是按照行来计算的
    double exposure_line_time;
    CameraGetExposureLineTime(h_camera_, &exposure_line_time);
    int exposure_time = params_.exposure_time;
    CameraSetExposureTime(h_camera_, exposure_time);
    std::cout << "Exposure time: " << exposure_time << std::endl;

    // Analog gain
    int analog_gain;
    CameraGetAnalogGain(h_camera_, &analog_gain);
    analog_gain = params_.gain;
    CameraSetAnalogGain(h_camera_, analog_gain);
    std::cout << "Analog gain: " << analog_gain << std::endl;

    // RGB Gain
    // Get default value
    CameraGetGain(h_camera_, &r_gain_, &g_gain_, &b_gain_);
    // R Gain
    double r_gain_ = params_.r_gain_;
    // G Gain
    double g_gain_ = params_.g_gain_;
    // B Gain
    double b_gain_ = params_.b_gain_;
    // Set gain
    CameraSetGain(h_camera_, r_gain_, g_gain_, b_gain_);
    std::cout << "RGB gain: " << r_gain_ << ", " << g_gain_ << ", " << b_gain_ << std::endl;

    // Saturation
    int saturation;
    CameraGetSaturation(h_camera_, &saturation);
    saturation = params_.saturation;
    CameraSetSaturation(h_camera_, saturation);
    std::cout << "Saturation: " << saturation << std::endl;

    // Gamma
    int gamma;
    CameraGetGamma(h_camera_, &gamma);
    gamma = params_.gamma;
    CameraSetGamma(h_camera_, gamma);
    std::cout << "Gamma: " << gamma << std::endl;

    // Flip
    flip_image_ = params_.flip_image;
  }
}  // namespace mindvision_camera

int main()
{
    mindvision_camera::MVCamera camera;
    while(true)
    {
        if(camera.over)
        {
            break;
        }
    }
    return 0;
}
