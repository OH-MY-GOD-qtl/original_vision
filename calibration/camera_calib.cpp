// 标定：棋盘格相机内参，退出时打印可直接粘贴回 auto_aim.yaml 的配置
#include <cfloat>
#include <cstdio>
#include <string>
#include <vector>

#include <opencv2/calib3d.hpp>
#include <opencv2/core/utility.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <yaml-cpp/yaml.h>

// 棋盘格的世界坐标：z=0 平面，单位 mm，所有图共用同一份
static std::vector<cv::Point3f> board_points(const cv::Size &pattern, float square_mm)
{
    std::vector<cv::Point3f> points;
    for (int row = 0; row < pattern.height; ++row)
        for (int col = 0; col < pattern.width; ++col)
            points.emplace_back(col * square_mm, row * square_mm, 0);
    return points;
}

// 打印成 yaml 的 [a, b, c] 形式
static void print_flow(const char *name, const cv::Mat &mat)
{
    const int n = mat.rows * mat.cols;
    std::printf("%s: [", name);
    for (int i = 0; i < n; ++i)
    {
        if (i > 0)
            std::printf(", ");
        std::printf("%.9g", mat.at<double>(i / mat.cols, i % mat.cols));
    }
    std::printf("]\n");
}

int main(int, char *argv[])
{
    const YAML::Node yaml = YAML::LoadFile(argv[1]);
    const std::string folder = argv[2];

    const cv::Size pattern(yaml["pattern_cols"].as<int>(), yaml["pattern_rows"].as<int>());
    const float square_mm = yaml["square_mm"].as<float>();

    std::vector<std::string> files;
    cv::glob(folder + "/*.jpg", files);

    const std::vector<cv::Point3f> board = board_points(pattern, square_mm);
    std::vector<std::string> used; // 识别成功的图，与 object_points 一一对应
    std::vector<std::vector<cv::Point3f>> object_points;
    std::vector<std::vector<cv::Point2f>> image_points;

    cv::Size image_size;
    const int flags = cv::CALIB_CB_ADAPTIVE_THRESH | cv::CALIB_CB_NORMALIZE_IMAGE;
    for (const auto &path : files)
    {
        const cv::Mat gray = cv::imread(path, cv::IMREAD_GRAYSCALE);
        image_size = gray.size();

        std::vector<cv::Point2f> corners;
        if (!cv::findChessboardCorners(gray, pattern, corners, flags))
        {
            std::printf("没识别到棋盘格，跳过 %s\n", path.c_str());
            continue;
        }

        cv::cornerSubPix(gray, corners, {11, 11}, {-1, -1},
                         cv::TermCriteria(cv::TermCriteria::EPS + cv::TermCriteria::MAX_ITER, 30,
                                          0.001));
        used.push_back(path);
        object_points.push_back(board);
        image_points.push_back(corners);
    }

    if (object_points.empty())
    {
        std::printf("没有可用的标定图\n");
        return 1;
    }

    // 视场角小，不需要 k3；默认 30 次迭代有时发散，放宽到 100
    const auto criteria =
        cv::TermCriteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 100, DBL_EPSILON);
    cv::Mat camera_matrix, distort;
    std::vector<cv::Mat> rvecs, tvecs;
    cv::calibrateCamera(object_points, image_points, image_size, camera_matrix, distort, rvecs,
                        tvecs, cv::CALIB_FIX_K3, criteria);

    // 每张图的重投影误差，明显偏大的那张删掉重算
    double error_sum = 0;
    size_t total_points = 0;
    for (size_t i = 0; i < object_points.size(); ++i)
    {
        std::vector<cv::Point2f> projected;
        cv::projectPoints(object_points[i], rvecs[i], tvecs[i], camera_matrix, distort, projected);

        double sum = 0;
        for (size_t j = 0; j < projected.size(); ++j)
            sum += cv::norm(image_points[i][j] - projected[j]);
        std::printf("%.3f px  %s\n", sum / projected.size(), used[i].c_str());

        error_sum += sum;
        total_points += projected.size();
    }

    std::printf("\n# 复制到 auto_aim.yaml（%zu 张，平均重投影误差 %.4f px）\n",
                object_points.size(), error_sum / total_points);
    print_flow("camera_matrix", camera_matrix);
    print_flow("distort_coeffs", distort);

    return 0;
}
