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


static bool insideTriangle(int x, int y, const Vector3f* _v)
{   
    // TODO : Implement this function to check if the point (x, y) is inside the triangle represented by _v[0], _v[1], _v[2]

     // Triangle's three side show in the vector way
    Eigen::Vector3f ab = _v[1] - _v[0];
    Eigen::Vector3f bc = _v[2] - _v[1];
    Eigen::Vector3f ca = _v[0] - _v[2];

    // judge point
    Eigen::Vector3f point = {(float)x,(float)y,_v[0].z()};
        
    // point to the Triangle three vertex show in the vector way
    Eigen::Vector3f ap = point - _v[0];
    Eigen::Vector3f bp = point - _v[1];
    Eigen::Vector3f cp = point - _v[2];


    Eigen::Vector3f ab_ap = ab.cross(ap);
    Eigen::Vector3f bc_bp = bc.cross(bp);
    Eigen::Vector3f ca_cp = ca.cross(cp);



    if(ab_ap.dot(bc_bp) >= 0 && bc_bp.dot(ca_cp) >=0  && ca_cp.dot(ab_ap) >= 0)
    {
        return true;
    }


    return false;
}

static std::tuple<float, float, float> computeBarycentric2D(float x, float y, const Vector3f* v)
{
    float c1 = (x*(v[1].y() - v[2].y()) + (v[2].x() - v[1].x())*y + v[1].x()*v[2].y() - v[2].x()*v[1].y()) / (v[0].x()*(v[1].y() - v[2].y()) + (v[2].x() - v[1].x())*v[0].y() + v[1].x()*v[2].y() - v[2].x()*v[1].y());
    float c2 = (x*(v[2].y() - v[0].y()) + (v[0].x() - v[2].x())*y + v[2].x()*v[0].y() - v[0].x()*v[2].y()) / (v[1].x()*(v[2].y() - v[0].y()) + (v[0].x() - v[2].x())*v[1].y() + v[2].x()*v[0].y() - v[0].x()*v[2].y());
    float c3 = (x*(v[0].y() - v[1].y()) + (v[1].x() - v[0].x())*y + v[0].x()*v[1].y() - v[1].x()*v[0].y()) / (v[2].x()*(v[0].y() - v[1].y()) + (v[1].x() - v[0].x())*v[2].y() + v[0].x()*v[1].y() - v[1].x()*v[0].y());
    return {c1,c2,c3};
}

void rst::rasterizer::draw(pos_buf_id pos_buffer, ind_buf_id ind_buffer, col_buf_id col_buffer, Primitive type,bool openSSAA,bool openMSAA)
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

        rasterize_triangle(t,openSSAA,openMSAA);
    }
}

//Screen space rasterization
void rst::rasterizer::rasterize_triangle(const Triangle& t,bool ssaa ,bool msaa) 
{
    auto v = t.toVector4();
    



    // fin out the bounding box of current triangle
    // float will have some wrong so we use int to instead 
    float minX,minY,maxX,maxY;

    minX = std::min(v[0].x(),std::min(v[1].x(),v[2].x()));
    minY = std::min(v[0].y(),std::min(v[1].y(),v[2].y()));

    maxX = std::max(v[0].x(),std::max(v[1].x(),v[2].x()));
    maxY = std::max(v[0].y(),std::max(v[1].y(),v[2].y()));


    // open ssaa
    if(ssaa == true)
    {
        std::vector<float> depth_buf_ssaa;
        std::vector<Eigen::Vector3f> frame_buf_ssaa;

        frame_buf_ssaa.resize(width * 2 * height * 2);
        depth_buf_ssaa.resize(width * 2 * height * 2);

        std::fill(depth_buf_ssaa.begin(), depth_buf_ssaa.end(), std::numeric_limits<float>::infinity());
        std::fill(frame_buf_ssaa.begin(), frame_buf_ssaa.end(), Eigen::Vector3f{0, 0, 0});

        minX *= 2;
        minY *= 2;
        maxY *= 2;
        maxX *= 2;

        Eigen::Vector3f vertices[] = {
            {v[0].x() * 2, v[0].y() * 2, v[0].z()},
            {v[1].x() * 2, v[1].y() * 2, v[1].z()},
            {v[2].x() * 2, v[2].y() * 2, v[2].z()}
        };
        
        for (int i = minX; i <= maxX; i++)
        {
            for ( int j = minY; j <= maxY; j++)
            {
                if(insideTriangle(i,j,vertices))
                {
                    auto tup = computeBarycentric2D(i + 0.5, j + 0.5, vertices);

                    float alpha, beta, gamma;
                    std::tie(alpha, beta, gamma) = tup;
                    float w_reciprocal = 1.0/(alpha / v[0].w() + beta / v[1].w() + gamma / v[2].w());
                

                    // In fact the v[0].w store the z value before thr pespective transform
                    float z_interpolated = alpha * v[0].z() / v[0].w() + beta * v[1].z() / v[1].w() + gamma * v[2].z() / v[2].w();
                    z_interpolated *= w_reciprocal;



                    if( depth_buf_ssaa[get_index_ssaa(i, j, 2)] > z_interpolated)
                    {
                        depth_buf_ssaa[get_index_ssaa(i, j, 2)] = z_interpolated;
                        frame_buf_ssaa[get_index_ssaa(i, j, 2)] = t.getColor();
                    }
                }
            }
        }

        for(int i = minX; i <= maxX; i += 2)
        {
            for(int j = minY; j <= maxY; j += 2)
            {
                Eigen::Vector3f point = {(float)(i / 2),(float)(j / 2) ,depth_buf_ssaa[get_index_ssaa(i, j, 2)]};
                Eigen::Vector3f color = frame_buf_ssaa[get_index_ssaa(i, j, 2)] 
                + frame_buf_ssaa[get_index_ssaa(i + 1, j, 2)] 
                + frame_buf_ssaa[get_index_ssaa(i + 1, j + 1, 2)] 
                + frame_buf_ssaa[get_index_ssaa(i , j + 1, 2)];

                color = color / 4;
                set_pixel(point,color);
            }
        }

    }
    // open msaa
    else if(msaa == true)
    {
        std::vector<float> depth_buf_msaa;

        depth_buf_msaa.resize(width * 2 * height * 2);

        std::fill(depth_buf_msaa.begin(), depth_buf_msaa.end(), std::numeric_limits<float>::infinity());

        minX *= 2;
        minY *= 2;
        maxY *= 2;
        maxX *= 2;

        Eigen::Vector3f vertices[] = {
            {v[0].x() * 2, v[0].y() * 2, v[0].z()},
            {v[1].x() * 2, v[1].y() * 2, v[1].z()},
            {v[2].x() * 2, v[2].y() * 2, v[2].z()}
        };
        
        for (int i = minX; i <= maxX; i++)
          {
            for ( int j = minY; j <= maxY; j++)
            {
                if(insideTriangle(i,j,vertices))
                {
                    auto tup = computeBarycentric2D(i + 0.5, j + 0.5, vertices);

                    float alpha, beta, gamma;
                    std::tie(alpha, beta, gamma) = tup;
                    float w_reciprocal = 1.0/(alpha / v[0].w() + beta / v[1].w() + gamma / v[2].w());
                

                    // In fact the v[0].w store the z value before thr pespective transform
                    float z_interpolated = alpha * v[0].z() / v[0].w() + beta * v[1].z() / v[1].w() + gamma * v[2].z() / v[2].w();
                    z_interpolated *= w_reciprocal;

                    if(depth_buf_msaa[get_index_ssaa(i, j, 2)] > z_interpolated)
                    {
                        depth_buf_msaa[get_index_ssaa(i, j, 2)] = z_interpolated;
                    }
                }
            }
        }


        for(int i = minX; i <= maxX; i += 2)
        {
            for(int j = minY; j <= maxY; j += 2)
            {


                int count = 0;
                float min_dep = std::numeric_limits<float>::infinity(); 
                
                float dep_1 = depth_buf_msaa[get_index_ssaa(i, j, 2)];
                float dep_2 = depth_buf_msaa[get_index_ssaa(i + 1, j, 2)];
                float dep_3 = depth_buf_msaa[get_index_ssaa(i, j + 1, 2)];
                float dep_4 = depth_buf_msaa[get_index_ssaa(i + 1, j + 1, 2)];


                if(min_dep != dep_1 && dep_1 == z_interpolated)
                {
                    count++;
                }
                if(min_dep != dep_2 && dep_2 == z_interpolated)
                {
                    count++;
                }
                if(min_dep != dep_3 && dep_3 == z_interpolated)
                {
                    count++;
                }
                if(min_dep != dep_4 && dep_4 == z_interpolated)
                {
                    count++;
                }
                Eigen::Vector3f point = {(float)(i / 2),(float)(j / 2), min_dep};
                set_pixel(point,t.getColor() * count / 4);


                // the code will have the depth problem 
                // at the same time, it will cause some problem that black content and don't know why
                // min_dep = std::min(dep_1,std::min(dep_2,std::min(dep_3,dep_4)));


                // if(min_dep != std::numeric_limits<float>::infinity())
                // {
                //     int count = 0;

                //     count += dep_1 == min_dep ? 1 : 0;
                //     count += dep_2 == min_dep ? 1 : 0;
                //     count += dep_3 == min_dep ? 1 : 0;
                //     count += dep_4 == min_dep ? 1 : 0;

                //     Eigen::Vector3f point = {(float)(i / 2),(float)(j / 2), min_dep};

                
                //     set_pixel(point,t.getColor() * count / 4);
                // }
            }
        }


    }
    // not open msaa and ssaa 
    else
    {     
        // makesure when back from ssaa or msaa the bounding box is right value.
        minX = std::min(v[0].x(),std::min(v[1].x(),v[2].x()));
        minY = std::min(v[0].y(),std::min(v[1].y(),v[2].y()));

        maxX = std::max(v[0].x(),std::max(v[1].x(),v[2].x()));
        maxY = std::max(v[0].y(),std::max(v[1].y(),v[2].y()));     

        for (int i = minX; i <= maxX; i++)
        {
            for ( int j = minY; j <= maxY; j++)
            {
            
                if(insideTriangle(i + 0.5,j + 0.5,t.v))
                {
                    auto tup = computeBarycentric2D(i, j, t.v);

                    float alpha, beta, gamma;
                    std::tie(alpha, beta, gamma) = tup;
                    float w_reciprocal = 1.0/(alpha / v[0].w() + beta / v[1].w() + gamma / v[2].w());
                

                    // In fact the v[0].w store the z value before thr pespective transform
                    float z_interpolated = alpha * v[0].z() / v[0].w() + beta * v[1].z() / v[1].w() + gamma * v[2].z() / v[2].w();
                    z_interpolated *= w_reciprocal;



                    if( depth_buf[get_index(i + 0.5,j + 0.5)] > z_interpolated)
                    {
                        depth_buf[get_index(i + 0.5,j+ 0.5)] = z_interpolated;
                        Eigen::Vector3f point = {(float)i,(float)j ,z_interpolated};
                        set_pixel(point,t.getColor());
                    }
                }
            }
        }
    }


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
    }
    if ((buff & rst::Buffers::Depth) == rst::Buffers::Depth)
    {
        std::fill(depth_buf.begin(), depth_buf.end(), std::numeric_limits<float>::infinity());
    }
}

rst::rasterizer::rasterizer(int w, int h) : width(w), height(h)
{
    frame_buf.resize(w * h);
    depth_buf.resize(w * h);
}

int rst::rasterizer::get_index(int x, int y)
{
    return (height-1-y)*width + x;
}

int rst::rasterizer::get_index_ssaa(int x, int y,int mulNum)
{
    int h = height * mulNum;
    int w = width * mulNum;
    return (h-1-y)*w + x;
}

void rst::rasterizer::set_pixel(const Eigen::Vector3f& point, const Eigen::Vector3f& color)
{
    //old index: auto ind = point.y() + point.x() * width;
    auto ind = (height-1-point.y())*width + point.x();
    frame_buf[ind] = color;

}

// clang-format on