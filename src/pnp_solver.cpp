#include "pnp_solver.hpp"

#include <algorithm>
#include <iostream>
#include <numeric>
#include <vector>

#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>

namespace
{
    // 六个面：顶点索引（按面的外轮廓顺序）+ 该面的颜色（BGR）。
    // 顶点编号约定见 cubeVertices()：
    //   0-3 正面（贴标签，z=0），4-7 背面（z=-cube_size）
    //   正面看：0 左上、1 右上、2 右下、3 左下
    struct Face
    {
        int v[4];
        cv::Scalar color;
    };

    const Face kFaces[6] = {
        // 0: 正面（贴标签那一面）—— 亮绿
        {{0, 1, 2, 3}, cv::Scalar(0, 255, 0)},
        // 1: 后面 —— 橙
        {{4, 5, 6, 7}, cv::Scalar(0, 165, 255)},
        // 2: 右侧面 —— 亮蓝
        {{1, 2, 6, 5}, cv::Scalar(255, 128, 0)},
        // 3: 左侧面 —— 亮蓝（左右同色）
        {{0, 3, 7, 4}, cv::Scalar(255, 128, 0)},
        // 4: 顶面 —— 洋红
        {{0, 1, 5, 4}, cv::Scalar(255, 0, 255)},
        // 5: 底面 —— 黄
        {{3, 2, 6, 7}, cv::Scalar(0, 255, 255)},
    };

    const char* const kFaceNames[6] = {"front", "back",  "right",
                                       "left",  "top",   "bottom"};

    // 填充透明度。0.2 表示 20% 不透明，太高会把标签糊掉。
    const double kAlpha = 0.2;

    // 描边线宽（像素）。
    const int kLineWidth = 2;

    // 面的四条边，用来画每个面的边框。
    // 顶点按外轮廓顺序给出，所以闭合折线就是这四条边。
    const int kEdgeCount = 4;
}

PnPSolver::PnPSolver(double tag_size, double cube_size)
    : tag_size_(tag_size),
      cube_size_(cube_size),
      calibrated_(false)
{
    // 内参先留空，等 loadCalibration 或 useFallbackIntrinsics 填。
    camera_matrix_ = cv::Mat::eye(3, 3, CV_64F);
    dist_coeffs_ = cv::Mat::zeros(5, 1, CV_64F);
}

bool PnPSolver::loadCalibration(const std::string& file)
{
    cv::FileStorage fs(file, cv::FileStorage::READ);
    if (!fs.isOpened())
    {
        std::cerr << "Cannot open calibration file: " << file << std::endl;
        return false;
    }

    fs["camera_matrix"] >> camera_matrix_;
    fs["distortion_coefficients"] >> dist_coeffs_;

    if (camera_matrix_.empty() || camera_matrix_.rows != 3 ||
        camera_matrix_.cols != 3)
    {
        std::cerr << "Invalid camera_matrix in: " << file << std::endl;
        return false;
    }

    if (dist_coeffs_.empty())
    {
        // 文件里没写畸变系数时按零畸变处理，不算错误。
        dist_coeffs_ = cv::Mat::zeros(5, 1, CV_64F);
    }

    // 统一成 double，避免后面 solvePnP 在 float 上做运算损失精度。
    if (camera_matrix_.type() != CV_64F)
    {
        camera_matrix_.convertTo(camera_matrix_, CV_64F);
    }
    if (dist_coeffs_.type() != CV_64F)
    {
        dist_coeffs_.convertTo(dist_coeffs_, CV_64F);
    }

    calibrated_ = true;
    return true;
}

void PnPSolver::useFallbackIntrinsics(const cv::Size& image_size)
{
    // 假设水平视场角约 60 度来估 fx。这个值只是让流程能跑，
    // 画出来的立方体大小和角度都不可信，不能用来验收精度。
    const double fx = image_size.width * 0.85;
    const double fy = fx;
    const double cx = image_size.width * 0.5;
    const double cy = image_size.height * 0.5;

    camera_matrix_ = (cv::Mat_<double>(3, 3) << fx, 0.0, cx,
                                               0.0, fy, cy,
                                               0.0, 0.0, 1.0);
    dist_coeffs_ = cv::Mat::zeros(5, 1, CV_64F);
    calibrated_ = false;
}

std::vector<cv::Point3f> PnPSolver::tagObjectPoints() const
{
    // 标签坐标系：x 向右、y 向上、z 从标签正面朝外（指向相机）。
    // 点序必须是 左上、右上、右下、左下，这是 IPPE_SQUARE 的硬性要求，
    // 也正好是 aruco 返回角点的顺序。
    const float h = static_cast<float>(tag_size_ * 0.5);

    std::vector<cv::Point3f> points;
    points.reserve(4);
    points.emplace_back(-h, h, 0.0F);   // 0 左上
    points.emplace_back(h, h, 0.0F);    // 1 右上
    points.emplace_back(h, -h, 0.0F);   // 2 右下
    points.emplace_back(-h, -h, 0.0F);  // 3 左下
    return points;
}

std::vector<cv::Point3f> PnPSolver::cubeVertices() const
{
    const float h = static_cast<float>(cube_size_ * 0.5);
    // 立方体实体在标签背面，所以 z 取负。
    const float z_back = static_cast<float>(-cube_size_);

    std::vector<cv::Point3f> vertices;
    vertices.reserve(8);

    // 正面（贴标签那一面），z = 0
    vertices.emplace_back(-h, h, 0.0F);        // 0 左上
    vertices.emplace_back(h, h, 0.0F);         // 1 右上
    vertices.emplace_back(h, -h, 0.0F);        // 2 右下
    vertices.emplace_back(-h, -h, 0.0F);       // 3 左下

    // 背面，xy 与正面相同，z 全部推到 -cube_size
    vertices.emplace_back(-h, h, z_back);      // 4
    vertices.emplace_back(h, h, z_back);       // 5
    vertices.emplace_back(h, -h, z_back);      // 6
    vertices.emplace_back(-h, -h, z_back);     // 7

    return vertices;
}

bool PnPSolver::solve(const std::vector<cv::Point2f>& corners,
                      cv::Vec3d& rvec,
                      cv::Vec3d& tvec) const
{
    if (corners.size() != 4)
    {
        return false;
    }

    // IPPE_SQUARE 是 4 个共面点专用的解法，比默认的迭代法稳，
    // 在平面目标上不会因为初值问题抖动。
    const bool ok = cv::solvePnP(tagObjectPoints(), corners, camera_matrix_,
                                 dist_coeffs_, rvec, tvec, false,
                                 cv::SOLVEPNP_IPPE_SQUARE);
    return ok;
}

std::vector<FaceDepth> PnPSolver::painterOrder(const cv::Vec3d& rvec,
                                               const cv::Vec3d& tvec) const
{
    cv::Mat rotation_mat;
    cv::Rodrigues(rvec, rotation_mat);
    // Mat 转成 3x3 定长矩阵，才能和 Vec3d 直接做乘法。
    const cv::Matx33d rotation(rotation_mat);

    const std::vector<cv::Point3f> vertices = cubeVertices();
    std::vector<FaceDepth> faces;
    faces.reserve(6);

    for (int face_index = 0; face_index < 6; ++face_index)
    {
        double depth_sum = 0.0;
        for (int k = 0; k < 4; ++k)
        {
            const cv::Point3f& p = vertices[static_cast<std::size_t>(kFaces[face_index].v[k])];
            // 标签坐标系 -> 相机坐标系：camera_point = R * p + t
            const cv::Vec3d camera_point =
                rotation * cv::Vec3d(p.x, p.y, p.z) + cv::Vec3d(tvec);
            depth_sum += camera_point(2);
        }

        FaceDepth face_depth;
        face_depth.index = face_index;
        face_depth.depth = depth_sum / 4.0;
        faces.push_back(face_depth);    
    }

    // 降序：z 大的（远）排前面，画家算法要求远的先画。
    // 写成升序就反了，近面会被远面盖住，看到"线跑到色块后面"的怪现象。
    std::sort(faces.begin(), faces.end(),
              [](const FaceDepth& a, const FaceDepth& b)
              { return a.depth > b.depth; });
        // lambda 表达式

    return faces;
}

const char* PnPSolver::faceName(int index)
{
    if (index < 0 || index >= 6)
    {
        return "?";
    }
    return kFaceNames[index];
}

void PnPSolver::drawCube(cv::Mat& image,
                         const cv::Vec3d& rvec,
                         const cv::Vec3d& tvec) const
{
    if (image.empty())
    {
        return;
    }

    // 8 个顶点一次性投影到像素坐标。projectPoints 会自动带上畸变，
    // 所以标定正确时直线会有极轻微的弯曲，那是正常现象。
    std::vector<cv::Point2f> projected;
    cv::projectPoints(cubeVertices(), rvec, tvec, camera_matrix_, dist_coeffs_,
                      projected);

    // 按画家顺序遍历：远的先画，近的后画盖住它。
    const std::vector<FaceDepth> order = painterOrder(rvec, tvec);

    for (const FaceDepth& face : order)
    {
        // projectPoints 给的是 Point2f，fillPoly/polylines 要 Point，
        // 所以这里逐个四舍五入成整数像素。
        std::vector<cv::Point> polygon;
        polygon.reserve(4);
        for (int k = 0; k < kEdgeCount; ++k)
        {
            const cv::Point2f& p =
                projected[static_cast<std::size_t>(kFaces[face.index].v[k])];
            polygon.emplace_back(cvRound(p.x), cvRound(p.y));
        }

        // 半透明填充：先画在不透明副本上，再按比例混合回来。
        // fillPoly 本身没有透明度参数，这是 OpenCV 里做半透明的标准写法。
        cv::Mat overlay = image.clone();
        cv::fillPoly(overlay, std::vector<std::vector<cv::Point>>{polygon},
                     kFaces[face.index].color, cv::LINE_AA);
        cv::addWeighted(overlay, kAlpha, image, 1.0 - kAlpha, 0.0, image);

        // 边框用同一个颜色画在不透明层上，这样线不会被自己的填充糊掉。
        cv::polylines(image, std::vector<std::vector<cv::Point>>{polygon}, true,
                      kFaces[face.index].color, kLineWidth, cv::LINE_AA);
    }
}

void PnPSolver::drawAxes(cv::Mat& image,
                         const cv::Vec3d& rvec,
                         const cv::Vec3d& tvec) const
{
    if (image.empty())
    {
        return;
    }

    // 坐标轴长度取立方体边长，这样箭头刚好伸到立方体边缘。
    // x 红、y 绿、z 蓝。
    // z 轴应该垂直穿出标签、朝相机方向指；如果它指向画面深处，说明位姿反了。
    cv::drawFrameAxes(image, camera_matrix_, dist_coeffs_, rvec, tvec,
                      static_cast<float>(cube_size_));
}
