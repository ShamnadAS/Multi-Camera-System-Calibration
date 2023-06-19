#pragma once
#include <Eigen/Dense>

class Camera
{
public:
	Eigen::Vector3d Position;
	Eigen::Matrix3d Orientation;
};