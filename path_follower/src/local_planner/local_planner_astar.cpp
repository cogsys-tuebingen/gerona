/// HEADER
#include <path_follower/local_planner/local_planner_astar.h>

/// PROJECT
#include <path_follower/pathfollower.h>

LocalPlannerAStar::LocalPlannerAStar(PathFollower &follower,
                                 tf::Transformer& transformer,
                                 const ros::Duration& update_interval)
    : LocalPlannerClassic(follower, transformer, update_interval)
{

}

void LocalPlannerAStar::printNodeUsage(int& nnodes) const{
    ROS_INFO_STREAM("# Nodes: " << nnodes);
}

int LocalPlannerAStar::algo(Eigen::Vector3d& pose, SubPath& waypoints, SubPath& local_wps,
                                  const std::vector<Constraint::Ptr>& constraints,
                                  const std::vector<Scorer::Ptr>& scorer,
                                  const std::vector<bool>& fconstraints,
                                  const std::vector<double>& wscorer,
                                  int& nnodes){
    // this planner uses the A* search algorithm
    initIndexes();
    initConstraintsAndScorers(constraints, scorer, fconstraints, wscorer, waypoints);

    const Waypoint& last = waypoints.back();
    const tf::Point wposep(pose(0),pose(1),pose(2));

    float dis2last = (wscorer.at(0) != 0.0)?global_path_.s(global_path_.n()-1):0.0;

    double heuristic = (dis2last - ((wscorer.at(0) != 0.0)?(wscorer.at(0)*scorer.at(0)->score(wposep)):0.0))
                            + ((wscorer.at(1) != 0.0)?(wscorer.at(1)*scorer.at(1)->score(wposep)):0.0)
                            + ((wscorer.at(2) != 0.0)?(wscorer.at(2)*scorer.at(2)->score(wposep)):0.0)
                            + ((fconstraints.at(1)?constraints.at(1)->isSatisfied(wposep):true)?
                                   ((wscorer.at(3) != 0.0)?(wscorer.at(3)*scorer.at(3)->score(wposep)):0.0):0.0);

    HNode wpose(pose(0),pose(1),pose(2),nullptr,0);
    wpose.gScore_ = 0.0;
    wpose.fScore_ = heuristic;

    if(dis2last - ((wscorer.at(0) != 0.0)?(wscorer.at(0)*scorer.at(0)->score(wposep)):0.0) < 0.8){
        return 0;
    }

    std::vector<HNode> nodes(200);
    nodes.at(0) = wpose;

    std::vector<HNode*> closedSet;

    prio_queue openSet;
    openSet.insert(&nodes[0]);
    double go_dist = std::numeric_limits<double>::infinity();
    HNode* obj = nullptr;
    int li_level = 10;
    nnodes = 1;

    HNode* current;

    while(!openSet.empty() && (openSet.empty()?nodes.back().level_:(*openSet.begin())->level_) <= li_level){
        current = *openSet.begin();
        openSet.erase(openSet.begin());
        if(isNearEnough(*current,last)){
            obj = current;
            break;
        }
        closedSet.push_back(current);

        std::vector<HNode*> successors;
        getSuccessors(current, nnodes, successors, nodes, constraints, fconstraints, true);
        for(std::size_t i = 0; i < successors.size(); ++i){
            if(std::find(closedSet.begin(), closedSet.end(), successors[i]) != closedSet.end()){
                continue;
            }

            double tentative_gScore = current->gScore_ + 0.15;//vllt tat. Abstand?

            if(tentative_gScore >= successors[i]->gScore_){
                continue;
            }

            successors[i]->parent_ = current;
            successors[i]->gScore_ = tentative_gScore;

            const tf::Point processed(successors[i]->x,successors[i]->y,
                    successors[i]->orientation);

            heuristic = (dis2last - ((wscorer.at(0) != 0.0)?(wscorer.at(0)*scorer.at(0)->score(processed)):0.0))
                    + ((wscorer.at(1) != 0.0)?(wscorer.at(1)*scorer.at(1)->score(processed)):0.0)
                    + ((wscorer.at(2) != 0.0)?(wscorer.at(2)*scorer.at(2)->score(processed)):0.0)
                    + ((fconstraints.at(1)?constraints.at(1)->isSatisfied(processed):true)?
                           ((wscorer.at(3) != 0.0)?(wscorer.at(3)*scorer.at(3)->score(processed)):0.0):0.0);


            successors[i]->fScore_ = heuristic;

            prio_queue::const_iterator inOpen = std::find(openSet.begin(), openSet.end(), successors[i]);
            if(inOpen == openSet.end()){
                openSet.insert(successors[i]);
            }else{
                openSet.erase(inOpen);
                openSet.insert(successors[i]);
            }

            if(heuristic < go_dist){
                go_dist = heuristic;
                obj = successors[i];
            }
        }
    }

    if(obj != nullptr){
        global_path_.set_s_new(global_path_.s_new() + 0.7);
        retrievePath(obj, local_wps);
        last_local_path_.assign(local_wps.begin(),local_wps.end());
        smoothAndInterpolate(local_wps);
        return 1;
    }else{
        return 0;
    }
}
