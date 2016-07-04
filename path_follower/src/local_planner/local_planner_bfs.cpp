/// HEADER
#include <path_follower/local_planner/local_planner_bfs.h>

/// PROJECT
#include <path_follower/pathfollower.h>

LocalPlannerBFS::LocalPlannerBFS(PathFollower &follower,
                                 tf::Transformer& transformer,
                                 const ros::Duration& update_interval)
    : LocalPlannerClassic(follower, transformer, update_interval)
{

}

bool LocalPlannerBFS::algo(Eigen::Vector3d& pose, SubPath& local_wps,
                                  const std::vector<Constraint::Ptr>& constraints,
                                  const std::vector<Scorer::Ptr>& scorer,
                                  const std::vector<bool>& fconstraints,
                                  const std::vector<double>& wscorer,
                                  int& nnodes){
    // this planner uses the Breadth-first search algorithm
    initIndexes();
    initScorers(scorer, wscorer);

    const Waypoint& last = waypoints.back();
    LNode wpose(pose(0),pose(1),pose(2),nullptr,0);

    float dis2last = (wscorer.at(0) != 0.0)?global_path_.s(global_path_.n()-1):0.0;

    if(dis2last + ((wscorer.at(0) != 0.0)?(wscorer.at(0)*scorer.at(0)->score(wpose)):0.0) < 0.8){
        tooClose = true;
        return false;
    }

    retrieveContinuity(wpose);
    setDistances(wpose,(fconstraints.at(1) || wscorer.at(4) != 0));

    std::vector<LNode> nodes(200);
    LNode* obj = nullptr;

    nodes.at(0) = wpose;

    std::queue<LNode*> fifo;
    fifo.push(&nodes[0]);
    double go_dist = std::numeric_limits<double>::infinity();
    int li_level = 10;
    nnodes = 1;

    LNode* current;

    while(!fifo.empty() && (fifo.empty()?nodes.back().level_:fifo.front()->level_) <= li_level){
        current = fifo.front();
        fifo.pop();
        if(isNearEnough(*current,last)){
            obj = current;
            break;
        }

        std::vector<LNode*> successors;
        getSuccessors(current, nnodes, successors, nodes, constraints, fconstraints, wscorer);
        for(std::size_t i = 0; i < successors.size(); ++i){
            double new_dist = Score(*(successors[i]), dis2last, scorer, wscorer);
            if(new_dist < go_dist){
                go_dist = new_dist;
                obj = successors[i];
            }
            fifo.push(successors[i]);
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
