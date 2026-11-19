#ifndef DETECTOR_HPP
#define DETECTOR_HPP

#include <vector>

#include <opencv2/aruco.hpp>
#include <opencv2/core.hpp>

// 一帧图像的检测结果
// ids和corners 一一对应：corners[i] 是编号 ids[i] 那个标签的 4 个角点
// 角点顺序由 aruco 保证：左上、右上、右下、左下，正好是 SOLVEPNP_IPPE_SQUARE要求的顺序
// 所以外部拿到之后可以直接交给 PnP，不需要重排
struct DetectionResult
{
    int count = 0;
    std::vector<int> ids;
    std::vector<std::vector<cv::Point2f>> corners;
};

// AprilTag 检测器
// 只负责"在图里找标签并把它画出来"，不碰位姿、不碰立方体
class Detector
{
public:
    Detector();

    // image        输入图像，彩色或灰度都接受
    // binary_image 输出：展示用二值图（灰度 + 固定阈值），不参与检测
    // result_image 输出：在原图副本上画好标签的结果图
    // 返回值        本帧检测的检测结果
    DetectionResult detect(const cv::Mat& image,
                           cv::Mat& binary_image,
                           cv::Mat& result_image,
                           int binary_threshold = 127) const;

private:
    cv::Ptr<cv::aruco::Dictionary> dictionary_;
    cv::Ptr<cv::aruco::DetectorParameters> parameters_;
};

#endif  // DETECTOR_HPP
