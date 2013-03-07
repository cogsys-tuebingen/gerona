/*
 * LaserTo3DTransformation.h
 *
 *  Created on: Nov 15, 2010
 *      Author: Michael Kaulig
 */

#ifndef LASERTO3DTRANSFORMATION_H_
#define LASERTO3DTRANSFORMATION_H_

#include "EigenTools.h"
#include <Eigen/Geometry>
#include "Global.h"

class LaserTo3DTransformation {
public:
	// Expects translation from middle of PanTilt Unit to middle of Laser
        LaserTo3DTransformation(Eigen::Vector3d trans);
	virtual ~LaserTo3DTransformation();

        void transform(double angle, double dist, const Eigen::Quaterniond& quat,const Eigen::Vector3d& laserPos, Eigen::Vector3d& res);

private:
        const Vector3d mTrans;
};

#endif /* LASERTO3DTRANSFORMATION_H_ */
