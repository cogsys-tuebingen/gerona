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
                                  std::size_t& nnodes){
    // this planner uses the Breadth-first search algorithm
    initIndexes(pose);

    LNode wpose(pose(0),pose(1),pose(2),nullptr,std::numeric_limits<double>::infinity(),0);
    setDistances(wpose,(fconstraints.back() || wscorer.back() != 0));

    float dis2last = global_path_.s(global_path_.n()-1);

    if(std::abs(dis2last - wpose.s) < 0.8){
        tooClose = true;
        setLLP();
        return false;
    }

    retrieveContinuity(wpose);
    setD2P(wpose);
    initConstraints(constraints,fconstraints);

    std::vector<LNode> nodes(nnodes_);
    LNode* obj = nullptr;

    setNormalizer(constraints,fconstraints);

    nodes.at(0) = wpose;

    std::queue<LNode*> fifo;
    fifo.push(&nodes[0]);
    initLeaves(nodes[0]);
    double best_p = std::numeric_limits<double>::infinity();
    int li_level = 10;
    nnodes = 1;

    LNode* current;

    while(!fifo.empty() && (fifo.empty()?nodes.at(nnodes - 1).level_:fifo.front()->level_) < li_level && nnodes < nnodes_){
        current = fifo.front();
        fifo.pop();
        if(std::abs(dis2last - current->s) <= 0.05){
            obj = current;
            tooClose = true;
            break;
        }

        std::vector<LNode*> successors;
        getSuccessors(current, nnodes, successors, nodes, constraints, fconstraints, wscorer);
        setNormalizer(constraints,fconstraints);
        updateLeaves(successors, current);
        for(std::size_t i = 0; i < successors.size(); ++i){
            // TODO: avoid to compute current_p if the tree is reconfigurable
            double current_p = Heuristic(*(successors[i]), dis2last) + Score(*(successors[i]), scorer, wscorer);
            updateBest(current_p,best_p,obj,successors[i]);
            fifo.push(successors[i]);
            addLeaf(successors[i]);
        }
    }
    reconfigureTree(obj, nodes, best_p, scorer, wscorer);
    if(obj != nullptr){
        return processPath(obj, local_wps);
    }else{
        return false;
    }
}
