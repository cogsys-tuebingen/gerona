/*
 * path_interpolated.h
 *
 *  Created on: Apr 25, 2015
 *      Author: holly
 */

#ifndef NAVIGATION_PATH_FOLLOWER_INCLUDE_PATH_FOLLOWER_UTILS_PATH_INTERPOLATED_H_
#define NAVIGATION_PATH_FOLLOWER_INCLUDE_PATH_FOLLOWER_UTILS_PATH_INTERPOLATED_H_

#include "path.h"
#include <nav_msgs/Path.h>

class PathInterpolated {
public:
	PathInterpolated();
	virtual ~PathInterpolated();

	void interpolatePath(const Path::Ptr path);

	inline double p(const double s) const {
		return p_[(int) s];
	}
	inline double q(const double s) const {
		return q_[(int) s];
	}

	inline double p_prim(const double s) const {
		return p_prim_[(int) s];
	}
	inline double q_prim(const double s) const {
		return q_prim_[(int) s];
	}

	inline double curvature(const double s) const {
		return curvature_[(int) s];
	}

	inline double theta_p(const double s) const {
		const int i = (int) s;
		return atan2(q_prim_[i], p_prim_[i]);
	}

	inline double p(const unsigned int s) const {
		return p_[s];
	}
	inline double q(const unsigned int s) const {
		return q_[s];
	}

	inline double p_prim(const unsigned int s) const {
		return p_prim_[s];
	}
	inline double q_prim(const unsigned int s) const {
		return q_prim_[s];
	}

	inline double curvature(const unsigned int s) const {
		return curvature_[s];
	}

	inline double theta_p(const unsigned int s) const {
		const int i = s;
		return atan2(q_prim_[i], p_prim_[i]);
	}

	inline unsigned int length() const {
		return p_.size();
	}

	operator nav_msgs::Path() const;

private:
	void clearBuffers();

	nav_msgs::Path interp_path;
	std::vector<double> p_;
	std::vector<double> q_;
	std::vector<double> p_prim_;
	std::vector<double> q_prim_;
	std::vector<double> curvature_;
};

#endif /* NAVIGATION_PATH_FOLLOWER_INCLUDE_PATH_FOLLOWER_UTILS_PATH_INTERPOLATED_H_ */
