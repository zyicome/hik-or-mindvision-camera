#include "MvCameraControl.h"
#include <iostream>
#include <thread>
#include <vector>
#include <unistd.h>

#include "opencv2/opencv.hpp"

namespace hik_camera
{
struct HikCameraParams
{
    double exposure_time = 5000;
    double gain = 16;
};

class HikCamera
{
public:
    explicit HikCamera();

    ~HikCamera();

    void declareParameters();

    std::vector<uint8_t> image_data_;
    cv::Mat image_;

    bool over = false;

    int nRet = MV_OK;
    void * camera_handle_;
    MV_IMAGE_BASIC_INFO img_info_;

    MV_CC_PIXEL_CONVERT_PARAM convert_param_;

    int fail_conut_ = 0;
    std::thread capture_thread_;

    HikCameraParams params_;
};

} // namespace hik_camera