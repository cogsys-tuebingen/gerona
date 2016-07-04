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

bool LocalPlannerAStar::algo(Eigen::Vector3d& pose, SubPath& local_wps,
                                  const std::vector<Constraint::Ptr>& constraints,
                                  const std::vector<Scorer::Ptr>& scorer,
                                  const std::vector<bool>& fconstraints,
                                  const std::vector<double>& wscorer,
                                  int& nnodes){
    // this planner uses the A* search algorithm
    initIndexes();
    initScorers(scorer, wscorer);

    const Waypoint& last = waypoints.back();
    HNode wpose(pose(0),pose(1),pose(2),nullptr,0);

    float dis2last = (wscorer.at(0) != 0.0)?global_path_.s(global_path_.n()-1):0.0;

    if(dis2last + ((wscorer.at(0) != 0.0)?(wscorer.at(0)*scorer.at(0)->score(wpose)):0.0) < 0.8){
        tooClose = true;
        return false;
    }

    retrieveContinuity(wpose);
    setDistances(wpose,(fconstraints.at(1) || wscorer.at(4) != 0));

    std::vector<HNode> nodes(200);
    HNode* obj = nullptr;

    double heuristic = Score(wpose, dis2last, scorer, wscorer);

    wpose.gScore_ = 0.0;
    wpose.fScore_ = heuristic;

    nodes.at(0) = wpose;

    std::vector<HNode*> closedSet;

    prio_queue openSet;
    openSet.insert(&nodes[0]);
    double go_dist = std::numeric_limits<double>::infinity();
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
        getSuccessors(current, nnodes, successors, nodes, constraints, fconstraints, wscorer/*, true*/);
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
            successors[i]->computeDiff();

            heuristic = Score(*(successors[i]), dis2last, scorer, wscorer);

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
        smoothAndInterpolate(local_wps);
        savePath(local_wps);
        return true;
    }else{
        return false;
    }
}
