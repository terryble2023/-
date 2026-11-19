#ifndef PNP_SOLVER_HPP
#define PNP_SOLVER_HPP

#include <string>
#include <vector>

#include <opencv2/core.hpp>

// 一个面在绘制顺序里的位置。
// index 是内部面表的下标，depth 是这个面 4 个顶点在相机坐标系下的平均 z（米）。
// z 越大表示离相机越远，画家算法要求远的先画。
struct FaceDepth
{
    int index = 0;
    double depth = 0.0;
};

// 位姿解算器。
// 负责：相机参数 -> PnP 解位姿 -> 把立方体投影回图像 -> 按面填色绘制。
class PnPSolver
{
public:
    // tag_size  标签实测边长（米），量黑框外沿，不是打印设置里的标称值
    // cube_size 要画的立方体边长（米），一般等于 tag_size
    PnPSolver(double tag_size, double cube_size);

    // 从 OpenCV 的 YAML/XML 标定文件读取内参和畸变系数。
    // 读取成功返回 true。文件里需要有 camera_matrix 和
    // distortion_coefficients 两个节点。
    bool loadCalibration(const std::string& file);

    // 没有标定文件时的内参。用作检验结果是否会出现语法报错。
    // 按图像尺寸猜一个 fx，只为了让整条流程能跑起来看效果，
    // 数值没有任何物理意义，画出来的立方体尺寸不可信。
    void useFallbackIntrinsics(const cv::Size& image_size);

    bool isCalibrated() const { return calibrated_; }

    // 用 4 个角点解出标签位姿。
    // corners 顺序必须是 左上、右上、右下、左下（aruco 的原始顺序）。
    // 解算失败或角点数不是 4 时返回 false。
    bool solve(const std::vector<cv::Point2f>& corners,
               cv::Vec3d& rvec,
               cv::Vec3d& tvec) const;

    // 把立方体画到 image 上：按面填半透明色 + 同色描边。
    void drawCube(cv::Mat& image,
                  const cv::Vec3d& rvec,
                  const cv::Vec3d& tvec) const;

    // 画坐标轴，用来肉眼验证 z 轴方向是否正确。
    void drawAxes(cv::Mat& image,
                  const cv::Vec3d& rvec,
                  const cv::Vec3d& tvec) const;

    // 返回 6 个面的绘制顺序，远的在前（画家算法要求的顺序）。
    // 公开出来是为了调试：可以打印它，转一圈标签，检查 front 是否永远最后。
    std::vector<FaceDepth> painterOrder(const cv::Vec3d& rvec,
                                        const cv::Vec3d& tvec) const;

    // 面的名字，用于调试打印。下标越界返回 "?"。
    static const char* faceName(int index);

private:
    // 标签 4 个角点在标签坐标系下的坐标，顺序与 solve 的输入一致。
    std::vector<cv::Point3f> tagObjectPoints() const;

    // 立方体 8 个顶点：0-3 正面（贴标签，z=0），4-7 背面（z=-cube_size）。
    std::vector<cv::Point3f> cubeVertices() const;

    cv::Mat camera_matrix_;
    cv::Mat dist_coeffs_;
    double tag_size_;
    double cube_size_;
    bool calibrated_;
};

#endif  // PNP_SOLVER_HPP
