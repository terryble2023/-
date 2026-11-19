#include "Detector.hpp"
#include "pnp_solver.hpp"

#include <iostream>
#include <string>
#include <vector>

#include <opencv2/highgui.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/videoio.hpp>

namespace
{
    // 标签实测边长，单位米。用尺子量打印出来的黑框外沿。
    // 换一张标签必须改这里，否则画出来的立方体大小不对。
    const double kTagSize = 0.04;

    // 要画的立方体边长。通常等于标签边长。
    const double kCubeSize = 0.04;

    // 打开摄像头后先丢弃的帧数。
    // 第一帧要等驱动完成格式协商，实测能到 1 秒以上，
    // 后面几帧曝光还在收敛，丢掉更稳。
    const int kDiscardFrames = 10;

    // 亮度低于这个值的帧直接跳过。摄像头启动或光线不足时
    // 会出这种帧，标签检测在里面基本不可能成功。
    const double kMinBrightness = 40.0;

    // 显示窗口的统一尺寸。只改显示大小，不动图像数据。
    const int kWindowWidth = 640;
    const int kWindowHeight = 360;

    bool isImagePath(const std::string& path)
    {
        const std::string image_extensions[] = {".png",  ".jpg",  ".jpeg",
                                                ".bmp",  ".tif",  ".tiff",
                                                ".webp"};
        const std::size_t dot = path.find_last_of('.');
        if (dot == std::string::npos)
        {
            return false;
        }

        std::string extension = path.substr(dot);
        for (char& ch : extension)
        {
            if (ch >= 'A' && ch <= 'Z')
            {
                ch = static_cast<char>(ch - 'A' + 'a');
            }
        }

        for (const std::string& known : image_extensions)
        {
            if (extension == known)
            {
                return true;
            }
        }
        return false;
    }

    // 处理一帧：检测 + 解位姿 + 画立方体，然后把三个窗口刷新出来。
    // 图片模式和摄像头模式共用这一段，避免两边逻辑各写一遍走偏。
    void processFrame(const cv::Mat& frame,
                      const Detector& detector,
                      const PnPSolver& solver,
                      bool show_axes)
    {
        cv::Mat binary_image;
        cv::Mat result_image;
        const DetectionResult detection =
            detector.detect(frame, binary_image, result_image);

        if (detection.count > 0)
        {
            cv::Vec3d rvec;
            cv::Vec3d tvec;
            // 只用第一个标签解位姿。多标签场景需要各自解一次，
            // 本作业只要求识别一个面，所以取第一个就够。
            if (solver.solve(detection.corners.front(), rvec, tvec))
            {
                solver.drawCube(result_image, rvec, tvec);
                if (show_axes)
                {
                    solver.drawAxes(result_image, rvec, tvec);
                }

                // 把距离写在画面上，方便肉眼判断解算结果合不合理。
                const double distance = cv::norm(tvec);
                const std::string info =
                    "dist: " + cv::format("%.3f", distance) + " m";
                cv::putText(result_image, info,
                            cv::Point(15, result_image.rows - 20),
                            cv::FONT_HERSHEY_SIMPLEX, 0.7,
                            cv::Scalar(0, 255, 255), 2, cv::LINE_AA);
            }
        }

        cv::imshow("Image_raw", frame);
        cv::imshow("Image_binary", binary_image);
        cv::imshow("result", result_image);
    }

    void createWindows()
    {
        // 显式创建成可缩放窗口，否则 imshow 自动建的是不可缩放的。
        const char* names[] = {"Image_raw", "Image_binary", "result"};
        for (const char* name : names)
        {
            cv::namedWindow(name, cv::WINDOW_NORMAL);
            cv::resizeWindow(name, kWindowWidth, kWindowHeight);
        }
    }

    int runImage(const std::string& path,
                 const Detector& detector,
                 PnPSolver& solver,
                 bool show_axes)
    {
        const cv::Mat image = cv::imread(path, cv::IMREAD_COLOR);
        if (image.empty())
        {
            std::cerr << "Cannot read image: " << path << std::endl;
            return 1;
        }

        // 图片模式下才知道图像尺寸，兜底内参在这里生成。
        if (!solver.isCalibrated())
        {
            solver.useFallbackIntrinsics(image.size());
            std::cout << "WARNING: using guessed intrinsics, "
                      << "cube size is not physically meaningful." << std::endl;
        }

        createWindows();
        processFrame(image, detector, solver, show_axes);

        std::cout << "Press any key to exit." << std::endl;
        cv::waitKey(0);
        cv::destroyAllWindows();
        return 0;
    }

    int runCamera(int camera_index,
                  const Detector& detector,
                  PnPSolver& solver,
                  bool show_axes)
    {
        // 必须显式指定 V4L2。用默认后端（CAP_ANY）在部分机器上会走
        // GStreamer，实测只能跑到 10 fps。
        cv::VideoCapture capture(camera_index, cv::CAP_V4L2);
        if (!capture.isOpened())
        {
            std::cerr << "Cannot open camera index " << camera_index
                      << std::endl;
            return 1;
        }

        capture.set(cv::CAP_PROP_FRAME_WIDTH, 640);
        capture.set(cv::CAP_PROP_FRAME_HEIGHT, 480);

        // 这两句是"尽力而为"：笔记本内置摄像头常常拒绝，
        // 返回 false 也不影响运行，所以不检查返回值。
        capture.set(cv::CAP_PROP_AUTO_EXPOSURE, 0.25);
        // 0.25为开关量，0.25为手动曝光，0.75为自动曝光。
        // 谁知道还有这种东西.
        capture.set(cv::CAP_PROP_EXPOSURE, 100);

        createWindows();

        // 预热：丢掉开头那些还没稳定的帧。
        cv::Mat frame;
        for (int i = 0; i < kDiscardFrames; ++i)
        {
            capture >> frame;
        }

        int frame_count = 0;
        while (true)
        {
            capture >> frame;
            if (frame.empty())
            {
                break;
            }

            // 太暗的帧跳过：曝光还没收敛或者环境太黑，检测不可能成功，
            // 强行跑只会让画面闪。这里仍要刷新一次窗口，
            // 否则窗口会卡住不响应。
            cv::Mat gray;
            cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);
            const double brightness = cv::mean(gray)[0];
            if (brightness < kMinBrightness)
            {
                cv::imshow("Image_raw", frame);
                if ((cv::waitKey(1) & 0xFF) == 27)
                {
                    break;
                }
                continue;
            }

            ++frame_count;
            processFrame(frame, detector, solver, show_axes);

            // waitKey(1) 而不是 waitKey(30)：帧率由摄像头决定，
            // 再睡 30ms 等于让延迟翻倍。
            const int key = cv::waitKey(1) & 0xFF;
            if (key == 27 || key == 'q' || key == 'Q')
            {
                break;
            }
        }

        std::cout << "Processed " << frame_count << " frames." << std::endl;
        capture.release();
        cv::destroyAllWindows();
        return 0;
    }
}

int main(int argc, char* argv[])
{
    if (argc < 2 || argc > 4)
    {
        std::cerr << "Usage: " << argv[0]
                  << " <image_path | camera_index> [calibration.yml] [--axes]"
                  << std::endl;
        std::cerr << "Examples:" << std::endl;
        std::cerr << "  " << argv[0] << " assets/tag.png camera.yml --axes"
                  << std::endl;
        std::cerr << "  " << argv[0] << " 0 camera.yml" << std::endl;
        return 1;
    }

    const std::string source = argv[1];

    // 解析可选参数：标定文件路径和 --axes 开关，顺序不限。
    std::string calibration_file;
    bool show_axes = false;
    for (int i = 2; i < argc; ++i)
    {
        const std::string arg = argv[i];
        if (arg == "--axes")
        {
            show_axes = true;
        }
        else
        {
            calibration_file = arg;
        }
    }

    Detector detector;
    PnPSolver solver(kTagSize, kCubeSize);

    if (!calibration_file.empty())
    {
        if (!solver.loadCalibration(calibration_file))
        {
            std::cerr << "Falling back to guessed intrinsics." << std::endl;
        }
    }

    if (isImagePath(source))
    {
        return runImage(source, detector, solver, show_axes);
    }

    if (!solver.isCalibrated())
    {
        solver.useFallbackIntrinsics(cv::Size(640, 480));
        std::cout << "WARNING: using guessed intrinsics, "
                  << "cube size is not physically meaningful." << std::endl;
    }

    int camera_index = 0;
    try
    {
        camera_index = std::stoi(source);
    }
    catch (const std::exception&)
    {
        std::cerr << "Source is neither an image path nor a camera index: "
                  << source << std::endl;
        return 1;
    }

    return runCamera(camera_index, detector, solver, show_axes);
}
