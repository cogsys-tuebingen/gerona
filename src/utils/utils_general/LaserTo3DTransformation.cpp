/*
 * LaserTo3DTransformation.cpp
 *
 *  Created on: Nov 15, 2010
 *      Author: Michael Kaulig
 */

#include "LaserTo3DTransformation.h"
#include <iostream>
#include <tf/tf.h>

LaserTo3DTransformation::LaserTo3DTransformation(Eigen::Vector3d trans): mTrans(trans){

}

LaserTo3DTransformation::~LaserTo3DTransformation() {
	// TODO Auto-generated destructor stub
}
/*
* @param angle: anlge of laser beam
* @param dist: measured range
* @param rpy: Orientation of Laserscanner
* @param trans: Translation from middle of Laser to middle of PanTilt Unit
* @param[out] res: transformed 3dpoint
*/
void LaserTo3DTransformation::transform(double angle, double dist, const Eigen::Quaterniond& quat,
                                        const Eigen::Vector3d& laserPos, Eigen::Vector3d& res){

	// Transforms a LaserscanPoint (angle, dist) to 3d Point in a frame in the Laser
        // PNI Koordinationsystem: x nach vorne (gegenüber Kabelaustritt) (Norden), y rechts (Osten) , z unten

        // berechne Karthesische Koordinaten im Laserscannerframe
        res = Eigen::Vector3d(dist * cos(angle),dist * sin(angle),0);

        // Translatiere in den Ursprung des Pan-Tilt-Frames
        res += mTrans;

        // rotiere um 180 deg um die x-Achse um ins Pni-Frame zu transformieren
        res[1] = -res[1];
        res[2] = -res[2];

        // rotiere um die Roll- und Tiltwinkel des PNI
        tf::Vector3 resBt;
        resBt[0] = res[0];
        resBt[1] = res[1];
        resBt[2] = res[2];
        tfScalar roll, pitch, yaw;
        tf::Quaternion quatBt;
        quatBt.setX(quat.x());
        quatBt.setY(quat.y());
        quatBt.setZ(quat.z());
        quatBt.setW(quat.w());

        tf::Matrix3x3(quatBt).getRPY(roll, pitch, yaw);

        tf::Matrix3x3 rotMatrix;
        rotMatrix.setRPY(roll, pitch, 0);
        resBt = rotMatrix * resBt;

        // Rotiere zurück ins Pan-Tilt-Koordinatenframe
        resBt[1] = -resBt[1];
        resBt[2] = -resBt[2];

        res[0] = resBt[0];
        res[1] = resBt[1];
        res[2] = resBt[2];

        // Translatiere zurück ins Laserkoordinatensystem
        res -= mTrans;

        // Translatiere ins Roboterkoordinatenframe
        res = res + laserPos;
}
