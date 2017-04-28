#ifndef LOCAL_PLANNER_PARAMETERS_H
#define LOCAL_PLANNER_PARAMETERS_H

#include <path_follower/utils/parameters.h>
#include <rosconsole/macros_generated.h>

struct LocalPlannerParameters : public Parameters
{
    //Parameters for the Local Planner
    P<std::string> local_planner;
    P<bool> c1, c2;
    P<double> s1, s2, s3, s4, s5, s6;
    P<int> nnodes,depth,ic,ia;
    P<double> uinterval,dis2p, adis, fdis, s_angle, lmf, mu, ef;
    P<bool> use_v;

    LocalPlannerParameters(Parameters* parent = nullptr):
        Parameters("local_planner", parent),

        local_planner(this, "algorithm", "AStar", "Algorithm to be used by the Local Planner."),

        //Constraints
        c1(this, "c1", true,
           "Determines whether the first constraint is used or not. (Distance to global path)"),
        c2(this, "c2", true,
           "Determines whether the second constraint is used or not. (Distance to nearest obstacle)"),

        //Scorers
        s1(this, "s1", 3.5,
           "Determines whether the first scorer is used or not. (Distance to global path (P))"),
        s2(this, "s2", 1.5,
           "Determines whether the second  scorer is used or not. (Distance to global path (D))"),
        s3(this, "s3", 2.0,
           "Determines whether the third scorer is used or not. (Curvature of the point (P))"),
        s4(this, "s4", 1.0,
           "Determines whether the fouth scorer is used or not. (Curvature of the point (D))"),
        s5(this, "s5", 1.5,
           "Determines whether the fifth scorer is used or not. (Tree level reached)"),
        s6(this, "s6", 5.0,
           "Determines whether the sixth scorer is used or not. (Distance to nearest obstacle)"),

        //Other local planner parameters
        nnodes(this, "nnodes", 400,
               "Determines the maximum number of nodes used by the local planner"),
        depth(this, "depth", 10,
               "Determines the maximum depth of the tree used by the local planner"),
        ic(this, "ic", 7,
               "Determines the number of intermediate configurations in a curve"),
        ia(this, "ia", 0,
               "Determines the number of intermediate angles between 0 and +-s_angle"),
        uinterval(this, "uinterval", 0.125,
                  "Determines the update interval in seconds of the local planner"),
        dis2p(this, "dis2p", 0.8,
              "Determines how far from the path should the local planner perform"),
        adis(this, "adis", 0.65,
              "Determines the security distance around the robot"),
        fdis(this, "fdis", 0.55,
              "Determines the extra security distance in front of the robot"),
        s_angle(this, "s_angle", 35.0,
                "Determines the steering angle (in degrees) for the local planner"),
        lmf(this, "lmf", 12.0,
                "Determines the multiplying factor of the intended length of the local path"),
        mu(this, "mu", 0.005,
           "Determines the coefficient of friction"),
        ef(this, "ef", 2.0,
           "Determines the multiplying factor for the argument of std::exp function in the dis2obst scorer"),
        use_v(this, "use_v", true,
              "Determines if the current velocity is used by the local planner")

    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    {
    }
};

#endif // LOCAL_PLANNER_PARAMETERS_H
