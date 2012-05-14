/**
 * @brief Only for testing some things.
 */
#include <stdio.h>
#include <ros/ros.h>
#include <Eigen/Core>
#include <Eigen/SVD>

using namespace std;
using namespace Eigen;

int main(int argc, char** argv)
{
    //ros::init(argc, argv, "test");
    //ros::NodeHandle node_handle;

    MatrixXf A(4,2);
    VectorXf b(4), x;

    A(0,0) = 1;
    A(1,0) = 2;
    A(2,0) = 3;
    A(3,0) = 4;
    A(0,1) = 1;
    A(1,1) = 1;
    A(2,1) = 1;
    A(3,1) = 1;
    b(0) = 1;
    b(1) = 3;
    b(2) = 2;
    b(3) = 4;


    //A.svd().solve(b, &x);
    JacobiSVD<MatrixXf> svd(A, ComputeThinU | ComputeThinV);
    x = svd.solve(b);

    cout << "a = " << x[0] << ", b = " << x[1] << endl;


    vector<int> foo(5);
    foo[0] = 1;
    foo[1] = 2;
    foo[2] = 3;
    foo[3] = 4;
    foo[4] = 5;

    cout << "FOO: ";
    for (vector<int>::iterator foo_it = foo.begin(); foo_it != foo.end(); ++foo_it) {
        cout << *foo_it << " ";
    }
    cout << endl;


    foo.pop_back();
    foo.pop_back();


    cout << "FOO: ";
    for (vector<int>::iterator foo_it = foo.begin(); foo_it != foo.end(); ++foo_it) {
        cout << *foo_it << " ";
    }
    cout << endl;

    return 0;
}

