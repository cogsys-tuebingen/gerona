/*
 * path_interpolated.cpp
 *
 *  Created on: Apr 25, 2015
 *      Author: holly
 */

// HEADER
#include <path_follower/utils/path_interpolated.h>

// THIRD PARTY
//#include <nav_msgs/Path.h>

// PROJECT
#include <path_follower/utils/cubic_spline_interpolation.h>
#include "../alglib/interpolation.h"
#include <utils_general/MathHelper.h>

// SYSTEM
#include <deque>
//#include <Eigen/Core>
//#include <Eigen/Dense>

using namespace Eigen;

PathInterpolated::PathInterpolated() {
}

PathInterpolated::~PathInterpolated() {
}

void PathInterpolated::interpolatePath(const Path::Ptr path) {

	clearBuffers();

	std::deque<Waypoint> waypoints;
	waypoints.insert(waypoints.end(), path->getCurrentSubPath().begin(),
			path->getCurrentSubPath().end());

	// (messy) hack!!!!!
	// TODO: why???
	// remove waypoints that are closer than 0.1 meters to the starting point
	Waypoint start = waypoints.front();
	while (!waypoints.empty()) {
		std::deque<Waypoint>::iterator it = waypoints.begin();
		const Waypoint& wp = *it;

		double dx = wp.x - start.x;
		double dy = wp.y - start.y;
		double distance = hypot(dx, dy);
		if (distance < 0.1) {
			waypoints.pop_front();
		} else {
			break;
		}
	}

	//copy the waypoints to arrays X_arr and Y_arr, and introduce a new array l_arr_unif required for the interpolation
	//as an intermediate step, calculate the arclength of the curve, and do the reparameterization with respect to arclength

	unsigned int N_ = waypoints.size();

	if (N_ < 2) {
		return;
	}

	double X_arr[N_], Y_arr[N_], l_arr_unif[N_];
	//double l_cum[N];
	double L = 0;

	//l_cum[0] = 0;
	for (std::size_t i = 1; i < N_; i++) {

		L += hypot(X_arr[i] - X_arr[i - 1], Y_arr[i] - Y_arr[i - 1]);
		//l_cum[i] = L;
	}

	double f = std::max(0.0001, L / (double) (N_ - 1));

	for (std::size_t i = 0; i < N_; ++i) {
		const Waypoint& waypoint = waypoints[i];

		X_arr[i] = waypoint.x;
		Y_arr[i] = waypoint.y;
		l_arr_unif[i] = i * f;

	}

	//initialization before the interpolation
	alglib::real_1d_array X_alg, Y_alg, l_alg_unif;

	X_alg.setcontent(N_, X_arr);
	Y_alg.setcontent(N_, Y_arr);
	l_alg_unif.setcontent(N_, l_arr_unif);

	alglib::spline1dinterpolant s_int1, s_int2;

	alglib::spline1dbuildcubic(l_alg_unif, X_alg, s_int1);
	alglib::spline1dbuildcubic(l_alg_unif, Y_alg, s_int2);

	//interpolate the path and find the derivatives, then publish the interpolated path
	for (uint i = 0; i < N_; ++i) {
		double x_s = 0.0, y_s = 0.0, x_s_prim = 0.0, y_s_prim = 0.0, x_s_sek =
				0.0, y_s_sek = 0.0;
		alglib::spline1ddiff(s_int1, l_alg_unif[i], x_s, x_s_prim, x_s_sek);
		alglib::spline1ddiff(s_int2, l_alg_unif[i], y_s, y_s_prim, y_s_sek);

		p_.push_back(x_s);
		q_.push_back(y_s);

		p_prim_.push_back(x_s_prim);
		q_prim_.push_back(y_s_prim);

		curvature_.push_back(
				(x_s_prim * y_s_sek - x_s_sek * y_s_prim)
						/ (sqrt(
								pow((x_s_prim * x_s_prim + y_s_prim * y_s_prim),
										3))));
	}

}

PathInterpolated::operator nav_msgs::Path() const {

	nav_msgs::Path path;
	const unsigned int length = p_.size();

	for (uint i = 0; i < length; ++i) {
		geometry_msgs::PoseStamped poza;
		poza.pose.position.x = p_[i];
		poza.pose.position.y = q_[i];
		path.poses.push_back(poza);
	}

	path.header.frame_id = "map";

	return path;
}

void PathInterpolated::clearBuffers() {
	p_.clear();
	q_.clear();

	p_prim_.clear();
	p_prim_.clear();

	curvature_.clear();
}
