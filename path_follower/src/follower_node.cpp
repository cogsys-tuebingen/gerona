#include <ros/ros.h>
#include <path_follower/pathfollower.h>
#include <path_follower/path_follower_server.h>
#include <path_follower/utils/parameters.h>
#include <path_follower/utils/obstacle_cloud.h>
#include <path_follower/utils/pose_tracker.h>
#include <path_follower/factory/follower_factory.h>
#include <pcl_ros/point_cloud.h>
#include <tf/tf.h>
#include <fstream>

namespace {
void importCloud(const ObstacleCloud::Cloud::ConstPtr& sensor_cloud, PathFollower* pf)
{
    ros::Time now;
    now.fromNSec(sensor_cloud->header.stamp * 1e3);

    std::string sensor_frame = sensor_cloud->header.frame_id;
    if(sensor_frame.at(0) == '/') {
        sensor_frame = sensor_frame.substr(1);
    }

    auto& pose_tracker = pf->getPoseTracker();

    try {
        tf::Transform fixed_to_sensor = pose_tracker.getRelativeTransform(pose_tracker.getFixedFrameId(), sensor_frame, now, ros::Duration(0.1));

        auto obstacle_cloud = std::make_shared<ObstacleCloud>(sensor_cloud);
        obstacle_cloud->transformCloud(fixed_to_sensor, pose_tracker.getFixedFrameId());
        pf->setObstacles(obstacle_cloud);
    } catch(const std::exception& e) {
        ROS_ERROR_STREAM_THROTTLE(1, "error transforming the obstacle cloud from " <<
                         sensor_frame << " to " <<
                         pose_tracker.getFixedFrameId() << ": " << e.what());
    }
}
}

void showHelp(const std::string& program)
{
    std::cout << "Usage:\n";
    std::cout << program << ": Run the path follower node.\n";
    std::cout << program << " --print: Show a list of parameters.\n";
    std::cout << program << " --dump [path]]: Write parameters to a file in markdown format.\n";
    std::cout << std::flush;
}

void printParameters()
{
    std::cout << "Path follower parameters:\n";
    Parameters::visitParameters([](const Parameters::ParamInfo& info) {
        std::cout << info.name << ":\t" << info.description << " (default: " << info.default_value << ")\n";
    });
    std::cout << std::flush;
}

int dumpParameters(PathFollower& pf, const std::string& filename)
{
    std::ofstream file;
    file.open(filename.c_str(), std::ios::out);

    if (!file.is_open()) {
        ROS_ERROR("Can't open file %s for writing.", filename.c_str());
        return 1;
    }

    ROS_INFO("generating parameter list");
    std::vector<std::shared_ptr<RobotController>> controllers;
    pf.getFollowerFactory().loadAll(controllers);

    std::vector<std::string> lines;
    Parameters::visitParameters([&file, &lines](const Parameters::ParamInfo& info) {
        std::stringstream line;
        line<< info.name << "\t| " << info.default_value << "\t| " << info.description;
        lines.push_back(line.str());
    });

    std::sort(lines.begin(), lines.end());

    file << "| Name | Default | Description |" << '\n';
    for(const std::string& line : lines) {
        file << line << '\n';
    }
    file.flush();
    file.close();

    ROS_INFO("Wrote parameters to %s.", filename.c_str());

    return 0;
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "path_follower_node", ros::init_options::NoSigintHandler);
    ros::NodeHandle nh;

    PathFollower pf(nh);
    PathFollowerServer server(pf);

    for(uint8_t i = 1; i < argc; ++i) {
        std::string arg = argv[i];
        if(arg == "--help" || arg == "-h") {
            // show usage
            showHelp(argv[0]);
            return 0;

        } else if(arg == "--print") {
            // list all parameters
            printParameters();
            return 0;

        } else if(arg == "--dump") {
            // print table of parameters
            std::string filename;
            if(i+1 < argc) {
                filename = argv[i+1];
            } else {
                filename = "/tmp/parameters.md";
            }

            return dumpParameters(pf, filename);
        }
    }


    ros::Subscriber obstacle_cloud_sub_ =
            nh.subscribe<ObstacleCloud::Cloud>("obstacle_cloud", 10,
                                        boost::bind(&importCloud, _1, &pf));

    server.spin();

    return 0;
}


