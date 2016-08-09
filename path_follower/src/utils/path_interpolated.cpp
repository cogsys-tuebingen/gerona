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

#include <nav_msgs/Path.h>

using namespace Eigen;

PathInterpolated::PathInterpolated()
	: N_(0),
      frame_id_("/map"),
	  s_new_(0),
	  s_prim_(0)
{
}

PathInterpolated::~PathInterpolated() {
}

void PathInterpolated::interpolatePath(const Path::Ptr path, const bool hack) {

	clearBuffers();

    frame_id_ = path->getFrameId();

	std::deque<Waypoint> waypoints;
	while (true) {
		waypoints.insert(waypoints.end(), path->getCurrentSubPath().begin(), path->getCurrentSubPath().end());

        if(hack){
            // (messy) hack!!!!!
            // remove waypoints that are closer than 0.1 meters to the starting point
            Waypoint start = waypoints.front();
            while(!waypoints.empty()) {
                std::deque<Waypoint>::iterator it = waypoints.begin();
                const Waypoint& wp = *it;

                double dx = wp.x - start.x;
                double dy = wp.y - start.y;
                double distance = hypot(dx, dy);
                if(distance < 0.1) {
                    waypoints.pop_front();
                } else {
                    break;
                }
            }

            // eliminate subpaths containing only the same points
            if(waypoints.size() > 0)
                break;

            path->switchToNextSubPath();
        }else{
            break;
        }
	}

	//copy the waypoints to arrays X_arr and Y_arr, and introduce a new array l_arr_unif required for the interpolation
	//as an intermediate step, calculate the arclength of the curve, and do the reparameterization with respect to arclength

	N_ = waypoints.size();

	if(N_ < 2) {
		return;
	}

	double X_arr[N_], Y_arr[N_], l_arr[N_], l_arr_unif[N_];
	double L = 0;

	for(std::size_t i = 0; i < N_; ++i) {
		const Waypoint& waypoint = waypoints[i];

		X_arr[i] = waypoint.x;
		Y_arr[i] = waypoint.y;

	}

	l_arr[0] = 0;

	for(std::size_t i = 1; i < N_; i++){

		L += hypot(X_arr[i] - X_arr[i-1], Y_arr[i] - Y_arr[i-1]);
		l_arr[i] = L;

	}
	ROS_INFO("Length of the path: %lf m", L);


	double f = std::max(0.0001, L / (double) (N_-1));

	for(std::size_t i = 1; i < N_; i++){

		l_arr_unif[i] = i * f;

	}

	//initialization before the interpolation
	alglib::real_1d_array X_alg, Y_alg, l_alg, l_alg_unif;
	alglib::real_1d_array x_s, y_s, x_s_prim, y_s_prim, x_s_sek, y_s_sek;

	X_alg.setcontent(N_, X_arr);
	Y_alg.setcontent(N_, Y_arr);
	l_alg.setcontent(N_, l_arr);
	l_alg_unif.setcontent(N_, l_arr_unif);


	//interpolate the path and find the derivatives
	alglib::spline1dconvdiff2cubic(l_alg, X_alg, l_alg_unif, x_s, x_s_prim, x_s_sek);
	alglib::spline1dconvdiff2cubic(l_alg, Y_alg, l_alg_unif, y_s, y_s_prim, y_s_sek);

	//define path components, its derivatives, and curvilinear abscissa, then calculate the path curvature
	for(uint i = 0; i < N_; ++i) {

		s_.push_back(l_arr_unif[i]);

		p_.push_back(x_s[i]);
		q_.push_back(y_s[i]);

		p_prim_.push_back(x_s_prim[i]);
		q_prim_.push_back(y_s_prim[i]);

		p_sek_.push_back(x_s_sek[i]);
		q_sek_.push_back(y_s_sek[i]);

		curvature_.push_back((x_s_prim[i]*y_s_sek[i] - x_s_sek[i]*y_s_prim[i])/
									(sqrt(pow((x_s_prim[i]*x_s_prim[i] + y_s_prim[i]*y_s_prim[i]), 3))));

	}

	assert(p_prim_.size() == N_);
	assert(q_prim_.size() == N_);
	assert(p_.size() == N_);
	assert(q_.size() == N_);
	assert(p_sek_.size() == N_);
	assert(q_sek_.size() == N_);
	assert(n() == N_);
}

double PathInterpolated::curvature_prim(const unsigned int i) const {
	if(n() <= 1)
		return 0.;

	unsigned int i_1 = i == n() - 1 ? i : i + 1;
	unsigned int i_0 = i_1 - 1;

	// differential quotient
	return (curvature(i_1) - curvature(i_0)) / (s(i_1) - s(i_0));
}


double PathInterpolated::curvature_sek(const unsigned int i) const {
	if(n() <= 2)
		return 0.;

	unsigned int i_1 = i == n() - 1 ? i : i + 1;
	unsigned int i_0 = i_1 - 1;

	// differential quotient
	return (curvature_prim(i_1) - curvature_prim(i_0)) / (s(i_1) - s(i_0));
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

    path.header.frame_id = frame_id_;

	return path;
}

PathInterpolated::operator SubPath() const {

    SubPath path(p_.size());
    const unsigned int length = p_.size();

    for (uint i = 0; i < length; ++i) {
        path.at(i).x = p_[i];
        path.at(i).y = q_[i];
        path.at(i).orientation = std::atan2(q_prim_[i],p_prim_[i]);
    }

    return path;
}

void PathInterpolated::clearBuffers() {
	N_ = 0;

	s_.clear();

	p_.clear();
	q_.clear();

	p_prim_.clear();
	q_prim_.clear();

	p_sek_.clear();
	q_sek_.clear();

	s_new_ = 0;
	s_prim_ = 0;

	curvature_.clear();

	interp_path.poses.clear();
}
