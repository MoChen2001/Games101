#include "Triangle.hpp"
#include "rasterizer.hpp"
#include <eigen3/Eigen/Eigen>
#include <iostream>
#include <opencv2/opencv.hpp>

constexpr double MY_PI = 3.1415926;



// You should notice that the vector is [1,1,1,1]' ,
// or we say column vector


Eigen::Matrix4f get_view_matrix(Eigen::Vector3f eye_pos)
{
    Eigen::Matrix4f view = Eigen::Matrix4f::Identity();

    Eigen::Matrix4f translate;
    translate << 1, 0, 0, -eye_pos[0], 0, 1, 0, -eye_pos[1], 0, 0, 1,
        -eye_pos[2], 0, 0, 0, 1;

    view = translate * view;

    return view;
}

Eigen::Matrix4f get_model_matrix(float rotation_angle ,float scaleFactor )
{
    Eigen::Matrix4f model = Eigen::Matrix4f::Identity();


    rotation_angle = rotation_angle * MY_PI / 180;



    // Rotation Matrix
    // Rotation around the aixs Z
    model << std::cos(rotation_angle), std::sin(rotation_angle),0,0,
    -1 * std::sin(rotation_angle), std::cos(rotation_angle),0,0,
    0,0,1,0,
    0,0,0,1;
    

    
    // Scale Matrix
    // Scalefactor will be premath in the main function 
    Eigen::Matrix4f scale = Eigen::Matrix4f::Identity();
    scale << scaleFactor,0,0,0,
    0,scaleFactor,0,0,
    0,0,scaleFactor,0,
    0,0,0,1;



    return model;// * scale;
}

Eigen::Matrix4f get_projection_matrix(float eye_fov, float aspect_ratio,
                                      float zNear, float zFar)
{
    Eigen::Matrix4f projection,m,p,n;

    zNear = -zNear;
    zFar = -zFar;  
    eye_fov = eye_fov * MY_PI / 180;
  
    float r,l,t,b;
    t = abs(std::tan(eye_fov / 2) * zNear);
    b = -1 * t;
    r = t * aspect_ratio;
    l = -r;


    n << 2 / (r - l), 0, 0, 0,\
    0, 2 /(t - b), 0, 0,\
    0, 0, 2 /(zNear - zFar), 0,\
    0,0,0,1;


    p << 1, 0, 0, -1 * (r + l) / 2,\
    0, 1, 0, -1 *(t + b) / 2,\
    0, 0, 1, -1 *(zFar + zNear) / 2,\
    0, 0, 0, 1;

    m << zNear,0,0,0, \
    0,zNear,0,0,\
    0,0,zNear + zFar, -1 * (zNear * zFar),\
    0,0,1,0;


    //std::cout <<  n * p * m;
    projection = n * p * m;

    return projection;
}

int main(int argc, const char** argv)
{
    float angle = 0;
    float scaleFactor = 0;
    bool command_line = false;
    std::string filename = "output.png";

    if (argc >= 3) {
        command_line = true;
        angle = std::stof(argv[2]); // -r by default
        if (argc == 4) {
            filename = std::string(argv[3]);
        }
        else
            return 0;
    }

    rst::rasterizer r(700, 700);

    Eigen::Vector3f eye_pos = {0, 0, 5};

    std::vector<Eigen::Vector3f> pos{{2, 0, -2}, {0, 2, -2}, {-2, 0, -2}};

    std::vector<Eigen::Vector3i> ind{{0, 1, 2}};

    auto pos_id = r.load_positions(pos);
    auto ind_id = r.load_indices(ind);

    int key = 0;
    int frame_count = 0;

    if (command_line) {
        r.clear(rst::Buffers::Color | rst::Buffers::Depth);

        r.set_model(get_model_matrix(angle,1));
        r.set_view(get_view_matrix(eye_pos));
        r.set_projection(get_projection_matrix(45, 1, 0.1, 50));

        r.draw(pos_id, ind_id, rst::Primitive::Triangle);
        cv::Mat image(700, 700, CV_32FC3, r.frame_buffer().data());
        image.convertTo(image, CV_8UC3, 1.0f);

        cv::imwrite(filename, image);

        return 0;
    }

    while (key != 27) {
        r.clear(rst::Buffers::Color | rst::Buffers::Depth);

        r.set_model(get_model_matrix(angle, 1 + std::sin(scaleFactor)));
        r.set_view(get_view_matrix(eye_pos));
        r.set_projection(get_projection_matrix(45, 1, 0.1, 50));

        r.draw(pos_id, ind_id, rst::Primitive::Triangle);

        cv::Mat image(700, 700, CV_32FC3, r.frame_buffer().data());
        image.convertTo(image, CV_8UC3, 1.0f);
        cv::imshow("image", image);
        key = cv::waitKey(10);

        std::cout << "frame count: " << frame_count++ <<'\n';

        angle -= 1;
        scaleFactor += 0.25;

        if (key == 'a' || key == 'A') {
            angle += 10;
        }
        else if (key == 'd'|| key == 'D') {
            angle -= 10;
        }
        
    }

    return 0;
}
