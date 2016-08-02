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
    initIndexes(pose);
    initScorers(scorer, wscorer);

    HNode wpose(pose(0),pose(1),pose(2),nullptr,0);

    float dis2last = (wscorer.at(0) != 0.0)?global_path_.s(global_path_.n()-1):0.0;

    if(dis2last + ((wscorer.at(0) != 0.0)?(wscorer.at(0)*scorer.at(0)->score(wpose)):0.0) < 0.8){
        tooClose = true;
        setLLP();
        return false;
    }

    retrieveContinuity(wpose);
    setDistances(wpose,(fconstraints.at(1) || wscorer.at(4) != 0));
    d2p = wpose.d2p;
    initConstraints(constraints,fconstraints);

    std::vector<HNode> nodes(nnodes_);
    HNode* obj = nullptr;

    double heuristic = Score(wpose, dis2last, scorer, wscorer);

    wpose.gScore_ = 0.0;
    wpose.fScore_ = heuristic;

    nodes.at(0) = wpose;

    std::vector<HNode*> closedSet;

    prio_queue openSet;
    openSet.insert(&nodes[0]);
    double go_dist = heuristic;
    int li_level = 10;
    nnodes = 1;

    HNode* current;

    while(!openSet.empty() && (openSet.empty()?nodes.at(nnodes - 1).level_:(*openSet.begin())->level_) <= li_level){
        current = *openSet.begin();
        openSet.erase(openSet.begin());
        if(std::abs(current->s - dis2last) <= 0.05){
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

            double tentative_gScore = current->gScore_ + step_;

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
        processPath(obj, local_wps);
        return true;
    }else{
        return false;
    }
}
