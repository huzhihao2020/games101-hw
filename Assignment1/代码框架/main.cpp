#include "Triangle.hpp"
#include "rasterizer.hpp"
#include <eigen3/Eigen/Eigen>
#include <iostream>
#include <opencv2/opencv.hpp>
//additional including
#include <cmath>

constexpr double MY_PI = 3.1415926;

Eigen::Matrix4f get_view_matrix(Eigen::Vector3f eye_pos)
{
    Eigen::Matrix4f view = Eigen::Matrix4f::Identity();

    Eigen::Matrix4f translate;
    translate << 1, 0, 0, -eye_pos[0], 0, 1, 0, -eye_pos[1], 0, 0, 1,
        -eye_pos[2], 0, 0, 0, 1;

    view = translate * view;
    std::clog << "view" << std::endl
              << view << std::endl;

    return view;
}

//rotaiton by z-axis
Eigen::Matrix4f get_model_matrix(float rotation_angle)
{
    Eigen::Matrix4f model = Eigen::Matrix4f::Identity();
    // TODO: Implement this function
    // Create the model matrix for rotating the triangle around the Z axis.
    // Then return it.
    // DONE
    float rotation_rad = acos(-1) * (rotation_angle / 180.0f);

    model << std::cos(rotation_rad), std::sin(-rotation_rad), 0, 0,
        std::sin(rotation_rad), std::cos(rotation_rad), 0, 0,
        0, 0, 1, 0,
        0, 0, 0, 1;

    std::clog << "model" << std::endl
              << model << std::endl;

    return model;
}

Eigen::Matrix4f get_projection_matrix(float eye_fov, float aspect_ratio,
                                      float zNear, float zFar)
{
    // Students will implement this function

    Eigen::Matrix4f projection = Eigen::Matrix4f::Identity();
    Eigen::Matrix4f M_o_trans, M_o_scale, M_p2o;
    // TODO: Implement this function
    // Create the projection matrix for the given parameters.
    // Then return it.
    // DONE

    float eye_fov_rad = acos(-1) * eye_fov / 180.0f;
    float t = std::abs(zNear) * std::tan(eye_fov_rad / 2.0f);
    float r = t * aspect_ratio;
    float b = -t;
    float l = -r;
    float n = -zNear;
    float f = -zFar;

    M_o_trans << 1, 0, 0, -(r + l) / 2.0f,
        0, 1, 0, -(t + b) / 2.0f,
        0, 0, 1, -(n + f) / 2.0f,
        0, 0, 0, 1;

    M_o_scale << 2.0f / (r - l), 0, 0, 0,
        0, 2.0f / (t - b), 0, 0,
        0, 0, 0, 2.0f / (n - f),
        0, 0, 0, 1;
    M_p2o << n, 0, 0, 0,
        0, n, 0, 0,
        0, 0, n + f, -(n * f),
        0, 0, 1, 0;

    projection = M_o_scale * M_o_trans * M_p2o;
    std::clog << "projection" << std::endl
              << projection << std::endl;
    return projection;
}

Eigen::Matrix4f get_rotation(Vector3f axis, float angle)
{
    Eigen::Matrix4f rotation = Eigen::Matrix4f::Identity();
    float rotation_rad = angle * MY_PI / 180.0f;
    Eigen::Matrix3f matrix_N;
    matrix_N << 0, -axis.z(), axis.y(),
        axis.z(), 0, -axis.x(),
        -axis.y(), axis.x(), 0;
    Eigen::Matrix3f rotation_3 = std::cos(rotation_rad) * Eigen::Matrix3f::Identity() + (1 - std::cos(rotation_rad)) * axis * axis.transpose() + std::sin(rotation_rad) * matrix_N;

    rotation.block<3, 3>(0, 0) = rotation_3;

    std::clog << "model_rotation_by_axis" << std::endl
              << rotation << std::endl;
    return rotation;
}
int main(int argc, const char **argv)
{
    float angle = 0;
    bool command_line = false;
    std::string filename = "output.png";

    if (argc >= 3)
    {
        command_line = true;
        angle = std::stof(argv[2]); // -r by default
        if (argc == 4)
        {
            filename = std::string(argv[3]);
        }
        else
            return 0;
    }

    rst::rasterizer r(700, 700);

    Eigen::Vector3f eye_pos = {0, 0, 5};

    std::vector<Eigen::Vector3f> pos{{2, 0, -2}, {0, 2, -2}, {-2, 0, -2}};
    //added by user
    // std::vector<Eigen::Vector3f> pos{{2, 0, -2}, {0, 2, -2}, {0, 0, -2}};
    // std::vector<Eigen::Vector3f> pos{{0, 0, -2}, {0, 2, -2}, {-2, 0, -2}};

    std::vector<Eigen::Vector3i> ind{{0, 1, 2}};

    auto pos_id = r.load_positions(pos);
    auto ind_id = r.load_indices(ind);

    int key = 0;
    int frame_count = 0;

    //added by user
    Eigen::Vector3f axis(0, 0, 1);

    if (command_line)
    {
        r.clear(rst::Buffers::Color | rst::Buffers::Depth);
        // r.set_model(get_model_matrix(angle));
        r.set_model(get_rotation(axis, angle));
        r.set_view(get_view_matrix(eye_pos));
        r.set_projection(get_projection_matrix(45, 1, 0.1, 50));

        r.draw(pos_id, ind_id, rst::Primitive::Triangle);
        cv::Mat image(700, 700, CV_32FC3, r.frame_buffer().data());
        image.convertTo(image, CV_8UC3, 1.0f);

        cv::imwrite(filename, image);

        return 0;
    }

    // 27 is the Ascii code of ESC
    while (key != 27)
    {
        r.clear(rst::Buffers::Color | rst::Buffers::Depth);
        // r.set_model(get_model_matrix(angle));
        r.set_model(get_rotation(axis, angle));
        r.set_view(get_view_matrix(eye_pos));
        r.set_projection(get_projection_matrix(45, 1, 0.1, 50));

        r.draw(pos_id, ind_id, rst::Primitive::Triangle);

        cv::Mat image(700, 700, CV_32FC3, r.frame_buffer().data());
        image.convertTo(image, CV_8UC3, 1.0f);
        cv::imshow("image", image);
        key = cv::waitKey(10);

        std::cout << "frame count: " << frame_count++ << '\n';
        std::cout << "angle: " << angle << std::endl;

        if (key == 'a')
        {
            angle += 10;
        }
        else if (key == 'd')
        {
            angle -= 10;
        }
    }

    return 0;
}
