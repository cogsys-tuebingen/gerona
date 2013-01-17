#ifndef GEOMETRYHELPER_H
#define GEOMETRYHELPER_H

#include "CircleSegment.h"
#include "LineSegment.h"

#include <boost/function.hpp>
#include <boost/bind.hpp>

namespace lib_path
{

class GeometryHelper
{
private:
    GeometryHelper();

public:
    /**
     * Computes a tangent, that touches this circle and the given one.
     * This computation takes into consideration the orientation and direction of both circles and the tangent.
     * The tangent fits onto both circles such that a continous trajectory (respecting the 3 directions) results.
     *
     * @param circle the second circle for the tangent
     */
    static bool get_common_tangent(CircleSegment &circle_from, CircleSegment& circle_to, LineSegment& tangent_out);



    /**
     * Computes a circle, that touches this circle and circle3 both tangentially.
     * This computation takes into consideration the orientation and direction of all three circles.
     * Since there are two possibilities, the combination with the lower weight is chosen.
     *
     * @param circle2 gets the information computed written into
     * @param circle3 the other circle, that must be touched
     */
    static bool get_tangential_circle(CircleSegment &circle_from, CircleSegment& circle_out, CircleSegment& circle_to, bool ignore_obstacles);

    /**
     * Computes two touching circles, that touch this circle and circle4 both tangentially.
     * This computation takes into consideration the orientation and direction of all four circles.
     * Since there are two possibilities, the combination with the lower weight is chosen.
     *
     * @param circle2 gets the information computed written into
     * @param circle3 gets the information computed written into
     * @param circle4 the other circle, that must be touched
     */
    static bool get_tangential_double_circle(CircleSegment &circle_from, CircleSegment& circle2, CircleSegment& circle3, CircleSegment& circle_to,
                                      bool ignore_obstacles);

private:
    static void external_tangent(CircleSegment& circle_to, LineSegment& tangent_out, CircleSegment& circle_from);
    static void internal_tangent(CircleSegment& circle_from, LineSegment& tangent_out, CircleSegment& circle_to);

    static bool symmetry_helper(const std::vector<CircleSegment *>& circles, double max_dist, bool ignore_obstacles,
                                boost::function<bool (const std::vector<CircleSegment *>&, bool)> callback);

    static bool test_tantential_circle_vector(const std::vector<CircleSegment*>& segments, bool choose_positive_solution);
    static bool test_tantential_circle(CircleSegment &circle_from, CircleSegment& circle2, CircleSegment& circle_to, bool choose_positive_solution);

    static bool test_tangential_double_circle_vector(const std::vector<CircleSegment*>& segments, bool choose_positive_solution);
    static bool test_tangential_double_circle(CircleSegment &circle_from, CircleSegment& circle2, CircleSegment& circle3, CircleSegment& circle_to,
                                                bool choose_positive_solution);

};

}

#endif // GEOMETRYHELPER_H
