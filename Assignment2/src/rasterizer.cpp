// clang-format off
//
// Created by goksu on 4/6/19.
//

#include <algorithm>
#include <vector>
#include "rasterizer.hpp"
#include <opencv2/opencv.hpp>
#include <math.h>


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


static bool insideTriangle(float x, float y, const Vector3f* _v)
{   
    // TODO : Implement this function to check if the point (x, y) is inside the triangle represented by _v[0], _v[1], _v[2]
    // Suppose _v[0] is point A, _v[1] is point B, _v[2] is point C
    // 0 means negative, 1 means positive
    int flag = -1;

    for(int i = 0; i < 3; i++) {
        // the current point
        Eigen::Vector3f p0(x, y, 0.0f);
        // the 1st vertex
        Eigen::Vector3f p1 = _v[i];
        // the 2nd vertex
        Eigen::Vector3f p2 = _v[(i+1)%3];
        
        // the 1st vector (p1-p0)
        Eigen::Vector3f v1 = p1-p0;
        // the 2nd vector (p1-p2)
        Eigen::Vector3f v2 = p1-p2;

        // get the cross product
        float cp = v1.cross(v2).z();
        if(cp == 0) continue;

        int sign = cp < 0 ? 0: 1;
        if(flag == -1) flag = sign;
        if(flag != sign) return false;
    }

    return true;

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

    Eigen::Matrix4f mvp = projection * view * model;
    for (auto& i : ind)
    {
        Triangle t;
        Eigen::Vector4f v[] = {
                mvp * to_vec4(buf[i[0]], 1.0f),
                mvp * to_vec4(buf[i[1]], 1.0f),
                mvp * to_vec4(buf[i[2]], 1.0f)
        };
        //Homogeneous division
        for (auto& vec : v) {
            vec /= vec.w();
        }
        //Viewport transformation
        for (auto & vert : v)
        {
            vert.x() = 0.5*width*(vert.x()+1.0);
            vert.y() = 0.5*height*(vert.y()+1.0);
            vert.z() = vert.z() * f1 + f2;
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

        rasterize_triangle(t);
    }
}

//Screen space rasterization
void rst::rasterizer::rasterize_triangle(const Triangle& t) {
    auto v = t.toVector4();

    // 2x2 超采样：每个像素在四个象限的中心各取一个样本
    static const float sample_offsets[4][2] = {
        {0.25f, 0.25f}, {0.75f, 0.25f}, {0.25f, 0.75f}, {0.75f, 0.75f}
    };

    float min_x_f = width;
    float max_x_f = 0;
    float min_y_f = height;
    float max_y_f = 0;

    // find out the bounding box of current triangle
    for(int i = 0; i < 3; i++) {
        min_x_f = std::min(v[i].x(), min_x_f);
        max_x_f = std::max(v[i].x(), max_x_f);
        min_y_f = std::min(v[i].y(), min_y_f);
        max_y_f = std::max(v[i].y(), max_y_f);
    }

    int min_x = std::max(0, static_cast<int>(std::floor(min_x_f)));
    int max_x = std::min(width - 1, static_cast<int>(std::ceil(max_x_f)));
    int min_y = std::max(0, static_cast<int>(std::floor(min_y_f)));
    int max_y = std::min(height - 1, static_cast<int>(std::ceil(max_y_f)));

    // iterate through the pixel
    for(int y = min_y; y <= max_y; y++) {
        for(int x = min_x; x <= max_x; x++) {

            int base_index = ((height - 1 - y) * width + x) * 4;
            bool touched = false;

            // 对每个样本分别做覆盖测试和深度测试
            for(int s = 0; s < 4; s++) {
                float sample_x = x + sample_offsets[s][0];
                float sample_y = y + sample_offsets[s][1];

                if(!insideTriangle(sample_x, sample_y, t.v)) {
                    continue;
                }

                // 用样本位置的屏幕坐标插值出该样本的深度
                auto[alpha, beta, gamma] = computeBarycentric2D(sample_x, sample_y, t.v);
                float w_reciprocal = 1.0/(alpha / v[0].w() + beta / v[1].w() + gamma / v[2].w());
                float z_interpolated = alpha * v[0].z() / v[0].w() + beta * v[1].z() / v[1].w() + gamma * v[2].z() / v[2].w();
                z_interpolated *= w_reciprocal;

                if(z_interpolated < sample_depth_buf[base_index + s]) {
                    sample_depth_buf[base_index + s] = z_interpolated;
                    sample_color_buf[base_index + s] = t.getColor();
                    touched = true;
                }
            }

            if(touched) {
                // 解析：四个样本的颜色取平均，得到该像素最终的颜色
                Eigen::Vector3f color(0.0f, 0.0f, 0.0f);
                float min_depth = std::numeric_limits<float>::infinity();
                for(int s = 0; s < 4; s++) {
                    color += sample_color_buf[base_index + s];
                    min_depth = std::min(min_depth, sample_depth_buf[base_index + s]);
                }
                color /= 4.0f;

                int pixel_index = (height - 1 - y) * width + x;
                depth_buf[pixel_index] = min_depth;

                Eigen::Vector3f p;
                p << x, y, min_depth;
                set_pixel(p, color);
            }
        }
    }


    // If so, use the following code to get the interpolated z value.
    // auto[alpha, beta, gamma] = computeBarycentric2D(x, y, t.v);
    // float w_reciprocal = 1.0/(alpha / v[0].w() + beta / v[1].w() + gamma / v[2].w());
    // float z_interpolated = alpha * v[0].z() / v[0].w() + beta * v[1].z() / v[1].w() + gamma * v[2].z() / v[2].w();
    // z_interpolated *= w_reciprocal;

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
        std::fill(sample_color_buf.begin(), sample_color_buf.end(), Eigen::Vector3f{0, 0, 0});
    }
    if ((buff & rst::Buffers::Depth) == rst::Buffers::Depth)
    {
        std::fill(depth_buf.begin(), depth_buf.end(), std::numeric_limits<float>::infinity());
        std::fill(sample_depth_buf.begin(), sample_depth_buf.end(), std::numeric_limits<float>::infinity());
    }
}

rst::rasterizer::rasterizer(int w, int h) : width(w), height(h)
{
    frame_buf.resize(w * h);
    depth_buf.resize(w * h);

    // 每个像素维护四个样本的颜色与深度
    sample_color_buf.resize(w * h * 4, Eigen::Vector3f{0, 0, 0});
    sample_depth_buf.resize(w * h * 4, std::numeric_limits<float>::infinity());
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

// clang-format on