#include <idtplanner.h>
#include <pi_astar_dt.h>
#include <pi_dwa_dt.h>
#include <pi_tree_dt.h>

IDTPlanner::Ptr IDTPlanner::Create(DTPlannerConfig &config)
{
    /*
    if (config.plannerType_ == "AStar_AngularVel_WSPL")
    {
        PI_AStar<NodeExpander_AVT_T,NodeScorer_Simple_T>::Ptr res = PI_AStar<NodeExpander_AVT_T,NodeScorer_Simple_T>::Create();
        res->Initialize(config);
        return res;
    }
    if (config.plannerType_ == "TreeDWA_AngularVel_WSPL")
    {
        PI_Tree<NodeExpander_AVNI_T,NodeScorer_Simple_T>::Ptr res = PI_Tree<NodeExpander_AVNI_T,NodeScorer_Simple_T>::Create();
        res->Initialize(config);
        return res;
    }
    if (config.plannerType_ == "DWA_AngularVel_WSPL")
    {
        PI_DWA<NodeExpander_AVT_T,NodeScorer_Simple_T>::Ptr res = PI_DWA<NodeExpander_AVT_T,NodeScorer_Simple_T>::Create();
        res->Initialize(config);
        return res;
    }
    */

    if (config.plannerType_ == "AStar")
    {
        if (config.scorerType_ == NodeScorer_GoalDT::NS_NAME)
        {
            PI_AStarDT<NodeScorer_GoalDT>::Ptr res = PI_AStarDT<NodeScorer_GoalDT>::Create();
            res->Initialize(config);
            return res;
        }
        if (config.scorerType_ == NodeScorer_PathDT::NS_NAME)
        {
            PI_AStarDT<NodeScorer_PathDT>::Ptr res = PI_AStarDT<NodeScorer_PathDT>::Create();
            res->Initialize(config);
            return res;
        }

        if (config.scorerType_ == NodeScorer_PathNGDT::NS_NAME)
        {
            PI_AStarDT<NodeScorer_PathNGDT>::Ptr res = PI_AStarDT<NodeScorer_PathNGDT>::Create();
            res->Initialize(config);
            return res;
        }
        // Org PI_AStar<NodeExpander_AVT_T,NodeScorer_Goal_T>::Ptr res = PI_AStar<NodeExpander_AVT_T,NodeScorer_Goal_T>::Create();

    }
    if (config.plannerType_ == "TreeDWA")
    {
        if (config.scorerType_ == NodeScorer_GoalDT::NS_NAME)
        {
            PI_TreeDT<NodeScorer_GoalDT>::Ptr res = PI_TreeDT<NodeScorer_GoalDT>::Create();
            res->Initialize(config);
            return res;
        }
        if (config.scorerType_ == NodeScorer_PathDT::NS_NAME)
        {
            PI_TreeDT<NodeScorer_PathDT>::Ptr res = PI_TreeDT<NodeScorer_PathDT>::Create();
            res->Initialize(config);
            return res;
        }
        if (config.scorerType_ == NodeScorer_PathNGDT::NS_NAME)
        {
            PI_TreeDT<NodeScorer_PathNGDT>::Ptr res = PI_TreeDT<NodeScorer_PathNGDT>::Create();
            res->Initialize(config);
            return res;
        }
    }
    if (config.plannerType_ == "DWA")
    {
        if (config.scorerType_ == NodeScorer_GoalDT::NS_NAME)
        {
            PI_DWADT<NodeScorer_GoalDT>::Ptr res = PI_DWADT<NodeScorer_GoalDT>::Create();
            res->Initialize(config);
            return res;
        }
        if (config.scorerType_ == NodeScorer_PathDT::NS_NAME)
        {
            PI_DWADT<NodeScorer_PathDT>::Ptr res = PI_DWADT<NodeScorer_PathDT>::Create();
            res->Initialize(config);
            return res;
        }
        if (config.scorerType_ == NodeScorer_PathNGDT::NS_NAME)
        {
            PI_DWADT<NodeScorer_PathNGDT>::Ptr res = PI_DWADT<NodeScorer_PathNGDT>::Create();
            res->Initialize(config);
            return res;
        }
    }

    /*
    if (config.plannerType_ == "AStar_LinAngVel_Goal_WSPL")
    {
        // Org PI_AStar<NodeExpander_AVT_T,NodeScorer_Goal_T>::Ptr res = PI_AStar<NodeExpander_AVT_T,NodeScorer_Goal_T>::Create();
        PI_AStar<NodeScorer_Goal_T>::Ptr res = PI_AStar<NodeScorer_Goal_T>::Create();
        res->Initialize(config);
        return res;
    }
    if (config.plannerType_ == "TreeDWA_LinAngVel_Goal_WSPL")
    {
        PI_Tree<NodeScorer_Goal_T>::Ptr res = PI_Tree<NodeScorer_Goal_T>::Create();
        res->Initialize(config);
        return res;
    }
    if (config.plannerType_ == "DWA_LinAngVel_Goal_WSPL")
    {
        PI_DWA<NodeScorer_Goal_T>::Ptr res = PI_DWA<NodeScorer_Goal_T>::Create();
        res->Initialize(config);
        return res;
    }
    */


    return nullptr;
}
