/**
 * @brief Only for testing some things.
 */
#include <stdio.h>
#include <ros/ros.h>
#include <Eigen/Core>
#include <Eigen/SVD>

using namespace std;
using namespace Eigen;


class LinearFunction
{
private:
    Vector2f coeff_;

public:
    LinearFunction(Vector2f coeff): coeff_(coeff) {}
    float a()
    {
        return coeff_[0];
    }
    float b()
    {
        return coeff_[1];
    }
    float operator()(float x)
    {
        return coeff_[0] * x + coeff_[1];
    }
};

int main(int argc, char** argv)
{
    //ros::init(argc, argv, "test");
    //ros::NodeHandle node_handle;

    Vector2f coeff(2, 1), right_coeff(1,1), left_coeff(32,145), mid_coeff;

//    LinearFunction f(coeff);

//    cout << "f(x) = " << f.a() << "*x + " << f.b() << endl;
//    cout << f(3) << endl;

    cout << right_coeff.norm() << endl;

    return 0;
}

