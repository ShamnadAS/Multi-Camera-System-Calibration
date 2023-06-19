#include <iostream>
#include <Eigen/Dense>
#include "src/calibration.hpp"

using Eigen::Vector3d;
using Eigen::Vector2d;

int main()
{
    Calibration calibration;

    Vector3d worldPos[] =
    {
        Vector3d(1, 1, 0), Vector3d(2, 1, 0), Vector3d(3, 1, 0),
        Vector3d(4, 1, 0), Vector3d(5, 1, 0), Vector3d(6, 1, 0)
    };

    Vector2d pixelPos[] =
    {
        Vector2d(1230, 1307), Vector2d(1378, 1312),
        Vector2d(1523, 1325), Vector2d(1677, 1336),
        Vector2d(1834, 1347), Vector2d(1994, 1360)
    };

    Eigen::MatrixXd projection = calibration.CalculateProjection(worldPos, pixelPos);
    calibration.CalculateExtrinsicProperties(projection);
}
