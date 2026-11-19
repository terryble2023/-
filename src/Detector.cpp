#include "Detector.hpp"

#include <string>

#include <opencv2/imgproc.hpp>

Detector::Detector()
// 依旧初始化
    : dictionary_(cv::aruco::getPredefinedDictionary(
          cv::aruco::DICT_APRILTAG_36h11)),
      parameters_(cv::aruco::DetectorParameters::create())
{
    // 角点精化用亚像素。从"整像素"到"亚像素"，PnP的位姿会稳很多。
    parameters_->cornerRefinementMethod = cv::aruco::CORNER_REFINE_SUBPIX;
}

DetectionResult Detector::detect(const cv::Mat& image,
                                 cv::Mat& binary_image,
                                 cv::Mat& result_image,
                                 int binary_threshold) const
{
    DetectionResult result;

    if (image.empty())
    {
        return result;
    }

    // 转灰度。detectMarkers 只接受单通道，彩色图必须先转。
    // 调用方如果已经给了灰度图，cvtColor也能正常处理。
    cv::Mat gray;
    if (image.channels() == 3)
    {
        cv::cvtColor(image, gray, cv::COLOR_BGR2GRAY);
    }
    else
    {
        gray = image;
    }

    // 展示用二值图。固定阈值，不参与检测。
    cv::threshold(gray, binary_image, binary_threshold, 255, cv::THRESH_BINARY);

    // 检测。注意这里传 gray 而不是 image。
    // 少一次内部转换，也避免彩色通道干扰。
    cv::aruco::detectMarkers(gray, dictionary_, result.corners, result.ids,
                             parameters_);
    result.count = static_cast<int>(result.ids.size());
    result_image = image.clone();

    if (result.count == 0)
    {
        cv::putText(result_image, "No AprilTag found", cv::Point(15, 40),
                    cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 0, 255), 2,
                    cv::LINE_AA);
        return result;
    }

    cv::aruco::drawDetectedMarkers(result_image, result.corners, result.ids);

    // 把每个角点画成红点、中心画成蓝点、旁边写编号。
    // 左上点画成绿色。
    for (int i = 0; i < result.count; ++i)
    {
        const std::vector<cv::Point2f>& tag_corners =
            result.corners[static_cast<std::size_t>(i)];

        if (tag_corners.size() != 4)
        {
            continue;
        }

        for (std::size_t j = 0; j < 4; ++j)
        // std::size_t 防止有无符号类型转换问题
        {
            const cv::Point pixel(cvRound(tag_corners[j].x),
                                  cvRound(tag_corners[j].y));
            // 0号角点用绿色标出来(其余为红色)，用来核对 IPPE_SQUARE 要求的起点位置。
            const cv::Scalar color = (j == 0) ? cv::Scalar(0, 255, 0)
                                              : cv::Scalar(0, 0, 255);
            cv::circle(result_image, pixel, 5, color, cv::FILLED, cv::LINE_AA);
        }

        const cv::Point2f center =
            (tag_corners[0] + tag_corners[1] + tag_corners[2] + tag_corners[3]) * 0.25F;
        const cv::Point center_pixel(cvRound(center.x), cvRound(center.y));
        cv::circle(result_image, center_pixel, 4, cv::Scalar(255, 0, 0),
                   cv::FILLED, cv::LINE_AA);

        const std::string label = "ID:" + std::to_string(result.ids[static_cast<std::size_t>(i)]);
        cv::putText(result_image, label,
                    cv::Point(center_pixel.x + 12, center_pixel.y - 12),
                    cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 255, 255), 2,
                    cv::LINE_AA);
    }

    const std::string status = "AprilTag detected: " + std::to_string(result.count);
    cv::putText(result_image, status, cv::Point(15, 40),
                cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 0, 255), 2,
                cv::LINE_AA);

    return result;
}
