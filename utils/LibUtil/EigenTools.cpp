/*
 * EigenTools.cpp
 *
 *  Created on: Aug 11, 2010
 *      Author: Sebastian Scherer <sebastian.scherer@uni-tuebingen.de>
 */

#include "EigenTools.h"

Eigen::Matrix2d RotMat2d(double phi) {
    double s = sin(phi);
    double c = cos(phi);
    Eigen::Matrix2d rot;
    rot << c, -s,
           s,  c;
    return rot;
}

Eigen::Matrix3d RotMatX(double phi) {
    Eigen::Matrix3d rot;
    rot << 1, 0,         0,
           0, cos(phi), -sin(phi),
           0, sin(phi),  cos(phi);
    return rot;
}

Eigen::Matrix3d RotMatY(double phi) {
    Eigen::Matrix3d rot;
    rot <<  cos(phi), 0, sin(phi),
            0       , 1, 0,
           -sin(phi), 0, cos(phi);
    return rot;
}

Eigen::Matrix3d RotMatZ(double phi) {
    Eigen::Matrix3d rot;
    rot << cos(phi), -sin(phi), 0,
           sin(phi),  cos(phi), 0,
           0       ,  0       , 1;
    return rot;
}

Eigen::Matrix3d RPYToMatrix(Eigen::Vector3d rpy)
{
    return RotMatZ(rpy[2])*RotMatY(rpy[1])*RotMatX(rpy[0]);
}
