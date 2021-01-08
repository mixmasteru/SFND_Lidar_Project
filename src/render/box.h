#ifndef BOX_H
#define BOX_H

#include <Eigen/Geometry>

struct BoxQ {
    Eigen::Vector3f bboxTransform;
    Eigen::Quaternionf bboxQuaternion;
    float cube_length;
    float cube_width;
    float cube_height;
};

struct Box {
    float x_min;
    float y_min;
    float z_min;
    float x_max;
    float y_max;
    float z_max;

    Box(){

    }

    Box(float x_min, float y_min, float z_min, float x_max, float y_max, float z_max) :
            x_min(x_min), y_min(y_min), z_min(z_min), x_max(x_max), y_max(y_max), z_max(z_max) {

    }
};

#endif