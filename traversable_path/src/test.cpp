/**
 * @brief Only for testing some things.
 */
#include <stdio.h>
//#include <ros/ros.h>
#include "pointclassification.h"
#include <boost/circular_buffer.hpp>

using namespace std;

int main(int argc, char** argv)
{
    //ros::init(argc, argv, "test");
    //ros::NodeHandle node_handle;

    cout << "testblub" << endl;

    boost::circular_buffer<int> blub(6);
//    blub.push_front(4);
//    blub.push_front(3);
//    blub.push_front(2);
//    blub.push_front(1);

    for (boost::circular_buffer<int> ::iterator it = blub.begin(); it != blub.end(); ++it) {
        cout << *it << "|";
    }
    cout << endl;

    int array[] = { 5, 6, 7, 8, 9 };
    blub.insert(blub.begin(), array, array + 5);

    blub.push_back(23);

    for (boost::circular_buffer<int> ::iterator it = blub.begin(); it != blub.end(); ++it) {
        cout << *it << "|";
    }
    cout << endl;

    return 0;
}

