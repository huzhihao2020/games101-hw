// clang-format off
//
// Created by goksu on 4/6/19.
//

#include <algorithm>
#include <vector>
#include "rasterizer.hpp"
#include <opencv2/opencv.hpp>
#include <math.h>

using namespace std;
#define MSAA 1

rst::pos_buf_id rst::rasterizer::load_positions(const std::vector<Eigen::Vector3f> &positions)
{
    auto id = get_next_id();
    pos_buf.emplace(id, positions);

    return {id};
}

rst::ind_buf_id rst::rasterizer::load_indices(const std::vector<Eigen::Vector3i> &indices)
{
    auto id = get_next_id();
    ind_buf.emplace(id, indices);

    return {id};
}

rst::col_buf_id rst::rasterizer::load_colors(const std::vector<Eigen::Vector3f> &cols)
{
    auto id = get_next_id();
    col_buf.emplace(id, cols);

    return {id};
}

auto to_vec4(const Eigen::Vector3f& v3, float w = 1.0f)
{
    return Vector4f(v3.x(), v3.y(), v3.z(), w);
}

//Modified to implement MSAA
// static bool insideTriangle(int x, int y, const Vector3f* _v)
static bool insideTriangle(float x, float y, const Vector3f* _v)
{   
    // TODO : Implement this function to check if the point (x, y) is inside the triangle represented by _v[0], _v[1], _v[2]
    // added by user, cross product
    // Done
    x += 0.5;
    y += 0.5;
    std::vector<Eigen::Vector2f> t_edge(3);
    Eigen::Vector2f point( x, y);
    t_edge[0] = (_v[1] - _v[0]).head<2>();
    t_edge[1] = (_v[2] - _v[1]).head<2>();
    t_edge[2] = (_v[0] - _v[2]).head<2>();
    
    float temp0 = t_edge[0].x()*(point - (_v[0]).head<2>()).y() - t_edge[0].y()*(point - (_v[0]).head<2>()).x();
    float temp1 = t_edge[1].x()*(point - (_v[1]).head<2>()).y() - t_edge[1].y()*(point - (_v[1]).head<2>()).x();
    float temp2 = t_edge[2].x()*(point - (_v[2]).head<2>()).y() - t_edge[2].y()*(point - (_v[2]).head<2>()).x();
    // std::cout << "temp0: "<< temp0  << " temp1: "<< temp1  << " temp2: "<< temp2 << std::endl;
    
    //p sameside
    bool isinside = ((temp0>=0)&&(temp1>=0)&&(temp2>=0))||((temp0<=0)&&(temp1<=0)&&(temp2<=0));

    return isinside;

}

static std::tuple<float, float, float> computeBarycentric2D(float x, float y, const Vector3f* v)
{
    float c1 = (x*(v[1].y() - v[2].y()) + (v[2].x() - v[1].x())*y + v[1].x()*v[2].y() - v[2].x()*v[1].y()) / (v[0].x()*(v[1].y() - v[2].y()) + (v[2].x() - v[1].x())*v[0].y() + v[1].x()*v[2].y() - v[2].x()*v[1].y());
    float c2 = (x*(v[2].y() - v[0].y()) + (v[0].x() - v[2].x())*y + v[2].x()*v[0].y() - v[0].x()*v[2].y()) / (v[1].x()*(v[2].y() - v[0].y()) + (v[0].x() - v[2].x())*v[1].y() + v[2].x()*v[0].y() - v[0].x()*v[2].y());
    float c3 = (x*(v[0].y() - v[1].y()) + (v[1].x() - v[0].x())*y + v[0].x()*v[1].y() - v[1].x()*v[0].y()) / (v[2].x()*(v[0].y() - v[1].y()) + (v[1].x() - v[0].x())*v[2].y() + v[0].x()*v[1].y() - v[1].x()*v[0].y());
    return {c1,c2,c3};
}

void rst::rasterizer::draw(pos_buf_id pos_buffer, ind_buf_id ind_buffer, col_buf_id col_buffer, Primitive type)
{
    auto& buf = pos_buf[pos_buffer.pos_id];
    auto& ind = ind_buf[ind_buffer.ind_id];
    auto& col = col_buf[col_buffer.col_id];

    float f1 = (50 - 0.1) / 2.0;
    float f2 = (50 + 0.1) / 2.0;

    // mvp : projection <- viewspace <- modelspace
    Eigen::Matrix4f mvp = projection * view * model;
    for (auto& i : ind)
    {
        Triangle t;
        Eigen::Vector4f v[] = {
                mvp * to_vec4(buf[i[0]], 1.0f),
                mvp * to_vec4(buf[i[1]], 1.0f),
                mvp * to_vec4(buf[i[2]], 1.0f)
        };

        //v.w() is exactly the z-value in view space
        for(auto idx : {0,1,2})
        {
            std::cout << "v" << idx << " | "<< v[idx].x()
                                    << " | "<< v[idx].y()
                                    << " | "<< v[idx].z()
                                    << " | "<< v[idx].w() << std::endl;
        }
        
        //Homogeneous division
        for (auto& vec : v) {
            vec /= vec.w();
        } 
    
        //Viewport transformation
        for (auto & vert : v)
        {
            vert.x() = 0.5*width*(vert.x()+1.0);
            vert.y() = 0.5*height*(vert.y()+1.0);
            vert.z() = -vert.z() * f1 + f2;
        }

        for (int i = 0; i < 3; ++i)
        {
            t.setVertex(i, v[i].head<3>());
            t.setVertex(i, v[i].head<3>());
            t.setVertex(i, v[i].head<3>());
        }

        auto col_x = col[i[0]];
        auto col_y = col[i[1]];
        auto col_z = col[i[2]];

        t.setColor(0, col_x[0], col_x[1], col_x[2]);
        t.setColor(1, col_y[0], col_y[1], col_y[2]);
        t.setColor(2, col_z[0], col_z[1], col_z[2]);
        
        //added by user
        std::cout << "Rasterizing triangle:" << i << std::endl;
        // std::cout << v[0] << std::endl;
        // std::cout << v[1] << std::endl;
        // std::cout << v[2] << std::endl;

        rasterize_triangle(t);
    }
}

//Screen space rasterization
void rst::rasterizer::rasterize_triangle(const Triangle& t) {
    // t 4x3 array, each col as a vertex
    auto v = t.toVector4();

    // TODO : Find out the bounding box of current triangle.
    // iterate through the pixel and find if the current pixel is inside the triangle
    //added by user

    //check vertex coordinates
    // for(auto idx : {0,1,2})
    // {
    //     std::cout << "pt" << idx << " | "<< v[idx].x()
    //                              << " | "<< v[idx].y()
    //                              << " | "<< v[idx].z()
    //                              << " | "<< v[idx].w() << std::endl;
    // }

    int x1, x2, y1, y2;
    x1 = std::floor(std::min(v[0].x(), std::min(v[1].x(), v[2].x())));
    x2 = std::ceil(std::max(v[0].x(), std::max(v[1].x(), v[2].x())));
    y1 = std::floor(std::min(v[0].y(), std::min(v[1].y(), v[2].y())));
    y2 = std::ceil(std::max(v[0].y(), std::max(v[1].y(), v[2].y())));

    //added by user
    // std::cout << "x1 x2 y1 y2: " << x1 << " " << x2 << " " << y1 << " " << y2 << std::endl;
    // std::cout << "v: " << std::endl << v[0] << std::endl << v[1]<< std::endl << v[2] << std::endl;
    //bounding box (x1, y1) (x2, y2)

    //applying Super sampling to Anti-aliasing
    if(MSAA)
    {
        for(int j = y2; j >= y1; j--)
        {
            for(int i = x1; i <= x2; i++)
            {
                int in_sum = 0;
                float pos[2] = {0.25, 0.75};
                
                for(auto x_sup:{0,1})
                {
                    for(auto y_sup:{0,1})
                    {
                        int sup_idx = get_index(i,j)*4 + x_sup*2 + y_sup;

                        auto[alpha, beta, gamma] = computeBarycentric2D(i+pos[x_sup], j+pos[y_sup], t.v);
                        float w_reciprocal = 1.0/(alpha / v[0].w() + beta / v[1].w() + gamma / v[2].w());
                        float z_interpolated = alpha * v[0].z() / v[0].w() + beta * v[1].z() / v[1].w() + gamma * v[2].z() / v[2].w();
                        z_interpolated *= w_reciprocal;

                        if(insideTriangle(i+pos[x_sup], j+pos[y_sup], t.v))
                        {
                            if(z_interpolated < sup_depthbuf[sup_idx])
                            {
                                sup_depthbuf[sup_idx] = z_interpolated;
                                in_sum++;
                            }
                            
                        }
                    }
                }

                if(in_sum>0)
                {
                    // If so, use the following code to get the interpolated z value.
          
                    // if lower z, update depth_buf

                    add_pixel(Vector3f(i, j, 1.0f), in_sum*t.getColor()/4.0f);

                    
                }
                    
            }
        }
    }
    // 不进行msaa，输出图像物体边缘会有锯齿感
    else
    {
        for(int j = y2; j >= y1; j--)
        {
            for(int i = x1; i <= x2; i++)
            {
                if(insideTriangle(i, j, t.v))
                {
                    // If so, use the following code to get the interpolated z value.
                    // std::cout << "(i,j) z_inter: " << z_interpolated << std::endl;
                    float alpha, beta, gamma;
                    std::tie(alpha, beta, gamma) = computeBarycentric2D(i, j, t.v);
                    float w_reciprocal = 1.0/(alpha / v[0].w() + beta / v[1].w() + gamma / v[2].w());
                    float z_interpolated = alpha * v[0].z() / v[0].w() + beta * v[1].z() / v[1].w() + gamma * v[2].z() / v[2].w();
                    z_interpolated *= w_reciprocal;

                    
                    // if lower z, update depth_buf
                    
                    if(z_interpolated < depth_buf[get_index(i, j)])
                    {
                        set_pixel(Vector3f(i, j, z_interpolated), t.getColor());
                        depth_buf[get_index(i, j)] = z_interpolated;
                    }
                    
                }
            }
        }
    }


    // TODO : set the current pixel (use the set_pixel function) to the color of the triangle (use getColor function) if it should be painted.
}

void rst::rasterizer::set_model(const Eigen::Matrix4f& m)
{
    model = m;
}

void rst::rasterizer::set_view(const Eigen::Matrix4f& v)
{
    view = v;
}

void rst::rasterizer::set_projection(const Eigen::Matrix4f& p)
{
    projection = p;
}

void rst::rasterizer::clear(rst::Buffers buff)
{
    if ((buff & rst::Buffers::Color) == rst::Buffers::Color)
    {
        std::fill(frame_buf.begin(), frame_buf.end(), Eigen::Vector3f{0, 0, 0});
        std::fill(sup_colorbuf.begin(), sup_colorbuf.end(), Eigen::Vector3f{0, 0, 0});
    }
    if ((buff & rst::Buffers::Depth) == rst::Buffers::Depth)
    {
        std::fill(depth_buf.begin(), depth_buf.end(), std::numeric_limits<float>::infinity());
        std::fill(sup_depthbuf.begin(), sup_depthbuf.end(), std::numeric_limits<float>::infinity());
    }
    
}

rst::rasterizer::rasterizer(int w, int h) : width(w), height(h)
{
    frame_buf.resize(w * h);
    depth_buf.resize(w * h);

    //added to use msaa
    sup_depthbuf.resize(w*h*4);
    sup_colorbuf.resize(w*h*4);
}

int rst::rasterizer::get_index(int x, int y)
{
    return (height-1-y)*width + x;
}

void rst::rasterizer::set_pixel(const Eigen::Vector3f& point, const Eigen::Vector3f& color)
{
    //old index: auto ind = point.y() + point.x() * width;
    auto ind = (height-1-point.y())*width + point.x();
    frame_buf[ind] = color;
}

void rst::rasterizer::add_pixel(const Eigen::Vector3f& point, const Eigen::Vector3f& color)
{
    //old index: auto ind = point.y() + point.x() * width;
    auto ind = (height-1-point.y())*width + point.x();
    frame_buf[ind] += color;
}
// clang-format on