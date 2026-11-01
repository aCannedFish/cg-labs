#include <algorithm>
#include <chrono>
#include <cmath>
#include <iostream>
#include <opencv2/opencv.hpp>

std::vector<cv::Point2f> control_points;

void mouse_handler(int event, int x, int y, int flags, void *userdata) 
{
    if (event == cv::EVENT_LBUTTONDOWN && control_points.size() < 4) 
    {
        std::cout << "Left button of the mouse is clicked - position (" << x << ", "
        << y << ")" << '\n';
        control_points.emplace_back(x, y);
    }     
}

void naive_bezier(const std::vector<cv::Point2f> &points, cv::Mat &window) 
{
    auto &p_0 = points[0];
    auto &p_1 = points[1];
    auto &p_2 = points[2];
    auto &p_3 = points[3];

    for (double t = 0.0; t <= 1.0; t += 0.001) 
    {
        auto point = std::pow(1 - t, 3) * p_0 + 3 * t * std::pow(1 - t, 2) * p_1 +
                 3 * std::pow(t, 2) * (1 - t) * p_2 + std::pow(t, 3) * p_3;

        window.at<cv::Vec3b>(point.y, point.x)[2] = 255;
    }
}

cv::Point2f recursive_bezier(const std::vector<cv::Point2f> &control_points, float t) 
{
    // TODO: Implement de Casteljau's algorithm
    // de Casteljau 算法：每一轮按比例 t 细分相邻控制点连成的线段，
    // 新得到的点序列长度减一；当序列只剩下一个点时，它就是曲线上参数为 t 的点。
    std::vector<cv::Point2f> points = control_points;
    for (int level = static_cast<int>(points.size()) - 1; level > 0; --level)
    {
        for (int i = 0; i < level; ++i)
        {
            points[i] = (1.0f - t) * points[i] + t * points[i + 1];
        }
    }
    return points[0];
}

void bezier(const std::vector<cv::Point2f> &control_points, cv::Mat &window) 
{
    // TODO: Iterate through all t = 0 to t = 1 with small steps, and call de Casteljau's 
    // recursive Bezier algorithm.
    // 以 0.001 为步长遍历 t，把 de Casteljau 求出的点写到绿色通道上。
    // 为了获得平滑的曲线，把颜色按距离分配给点周围最近的四个像素（反走样）。
    for (double t = 0.0; t <= 1.0; t += 0.001)
    {
        cv::Point2f point = recursive_bezier(control_points, static_cast<float>(t));

        int x0 = static_cast<int>(std::floor(point.x));
        int y0 = static_cast<int>(std::floor(point.y));
        for (int dx = 0; dx <= 1; ++dx)
        {
            for (int dy = 0; dy <= 1; ++dy)
            {
                int px = x0 + dx;
                int py = y0 + dy;
                if (px < 0 || px >= window.cols || py < 0 || py >= window.rows)
                {
                    continue;
                }
                // 像素中心到曲线点的距离越近，权重越大
                float weight = (1.0f - std::fabs(point.x - px)) * (1.0f - std::fabs(point.y - py));
                if (weight <= 0.0f)
                {
                    continue;
                }
                cv::Vec3b &pixel = window.at<cv::Vec3b>(py, px);
                pixel[1] = std::max(pixel[1], static_cast<uchar>(255.0f * weight));
            }
        }
    }
}

int main() 
{
    cv::Mat window = cv::Mat(700, 700, CV_8UC3, cv::Scalar(0));
    cv::cvtColor(window, window, cv::COLOR_BGR2RGB);
    cv::namedWindow("Bezier Curve", cv::WINDOW_AUTOSIZE);

    cv::setMouseCallback("Bezier Curve", mouse_handler, nullptr);

    int key = -1;
    while (key != 27) 
    {
        for (auto &point : control_points) 
        {
            cv::circle(window, point, 3, {255, 255, 255}, 3);
        }

        if (control_points.size() == 4) 
        {
            //   naive_bezier(control_points, window);
            bezier(control_points, window);

            cv::imshow("Bezier Curve", window);
            cv::imwrite("my_bezier_curve.png", window);
            key = cv::waitKey(0);

            return 0;
        }

        cv::imshow("Bezier Curve", window);
        key = cv::waitKey(20);
    }

return 0;
}
