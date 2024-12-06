// MindVision Camera SDK
#include <CameraApi.h>

// C++ system
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include "opencv2/opencv.hpp"

namespace mindvision_camera
{

struct MVCameraParams
{
    double exposure_time = 5000;
    double gain = 64;
    double r_gain_ = 100;
    double g_gain_ = 100;
    double b_gain_ = 100;
    int saturation = 128;
    int gamma = 100;
    bool flip_image = false;
};

class MVCamera
{
public:
    explicit MVCamera();

    ~MVCamera();

    void declareParameters();

    std::vector<uint8_t> image_data_;
    cv::Mat image_;

    bool over = false;

    MVCameraParams params_;

    int h_camera_;
    uint8_t * pby_buffer_;
    tSdkCameraCapbility t_capability_;  // 设备描述信息
    tSdkFrameHead s_frame_info_;        // 图像帧头信息

    // RGB Gain
    int r_gain_, g_gain_, b_gain_;

    bool flip_image_;

    int fail_conut_ = 0;
    std::thread capture_thread_;
};

} // namespace mindvision_camera