//
// Created by LEI XU on 4/27/19.
//

#ifndef RASTERIZER_TEXTURE_H
#define RASTERIZER_TEXTURE_H
#include "global.hpp"
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
        auto u_img = u * width;
        auto v_img = (1 - v) * height;
        // std::cout << "u_img, v_img: " << u_img << v_img << std::endl;
        auto color = image_data.at<cv::Vec3b>(v_img, u_img);
        return Eigen::Vector3f(color[0], color[1], color[2]);
    }

    Eigen::Vector3f getColorBilinear(float u, float v)
    {
        auto u_img = u * width;
        auto v_img = (1 - v) * height;

        int u0 = std::floor(u_img);
        int u1 = std::ceil(u_img);
        int v0 = std::floor(v_img);
        int v1 = std::ceil(v_img);
        float ux = u_img - u0;
        float vx = v1 - v_img;

        auto color00 = image_data.at<cv::Vec3b>(v1, u0);
        auto color01 = image_data.at<cv::Vec3b>(v1, u1);

        auto color10 = image_data.at<cv::Vec3b>(v0, u0);
        auto color11 = image_data.at<cv::Vec3b>(v0, u1);
        // std::cout << "ux, vx: " << ux << vx << std::endl;

        //第一次
        auto color1 = color00 * (1-ux) + color01 * ux;
        auto color2 = color10 * (1-ux) + color11 * ux;
        //第二次
        auto color3 = color1 * (1-vx) + color2 * vx;

        return Eigen::Vector3f(color3[0], color3[1], color3[2]);
    }

};
#endif //RASTERIZER_TEXTURE_H
