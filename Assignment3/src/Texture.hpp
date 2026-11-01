//
// Created by LEI XU on 4/27/19.
//

#ifndef RASTERIZER_TEXTURE_H
#define RASTERIZER_TEXTURE_H
#include "global.hpp"
#include <algorithm>
#include <cmath>
#include <eigen3/Eigen/Eigen>
#include <opencv2/opencv.hpp>
class Texture{
private:
    cv::Mat image_data;

public:
    Texture(const std::string& name)
    {
        image_data = cv::imread(name);
        cv::cvtColor(image_data, image_data, cv::COLOR_RGB2BGR);
        width = image_data.cols;
        height = image_data.rows;
    }

    int width, height;

    Eigen::Vector3f getColor(float u, float v)
    {
        // 双线性插值：先把纹理坐标映射到纹素坐标，再对相邻四个纹素按距离加权
        u = std::clamp(u, 0.0f, 1.0f);
        v = std::clamp(v, 0.0f, 1.0f);

        float x = u * (width - 1);
        float y = (1.0f - v) * (height - 1);

        int x0 = static_cast<int>(std::floor(x));
        int y0 = static_cast<int>(std::floor(y));
        int x1 = std::min(x0 + 1, width - 1);
        int y1 = std::min(y0 + 1, height - 1);

        float fx = x - x0;
        float fy = y - y0;

        auto to_vector = [](const cv::Vec3b &color) {
            return Eigen::Vector3f(color[0], color[1], color[2]);
        };

        Eigen::Vector3f c00 = to_vector(image_data.at<cv::Vec3b>(y0, x0));
        Eigen::Vector3f c10 = to_vector(image_data.at<cv::Vec3b>(y0, x1));
        Eigen::Vector3f c01 = to_vector(image_data.at<cv::Vec3b>(y1, x0));
        Eigen::Vector3f c11 = to_vector(image_data.at<cv::Vec3b>(y1, x1));

        Eigen::Vector3f top = c00 * (1.0f - fx) + c10 * fx;
        Eigen::Vector3f bottom = c01 * (1.0f - fx) + c11 * fx;
        return top * (1.0f - fy) + bottom * fy;
    }

};
#endif //RASTERIZER_TEXTURE_H
