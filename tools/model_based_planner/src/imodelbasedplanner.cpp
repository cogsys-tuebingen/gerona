#include <imodelbasedplanner.h>
#include <pi_astar.h>
#include <pi_dwa.h>
#include <pi_tree.h>

IModelBasedPlanner::Ptr IModelBasedPlanner::Create(ModelBasedPlannerConfig &config)
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
        if (config.scorerType_ == NodeScorer_Goal_T::NS_NAME)
        {
            PI_AStar<NodeScorer_Goal_T>::Ptr res = PI_AStar<NodeScorer_Goal_T>::Create();
            res->Initialize(config);
            return res;
        }
        if (config.scorerType_ == NodeScorer_Path_T::NS_NAME)
        {
            PI_AStar<NodeScorer_Path_T>::Ptr res = PI_AStar<NodeScorer_Path_T>::Create();
            res->Initialize(config);
            return res;
        }

        if (config.scorerType_ == NodeScorer_PathNG_T::NS_NAME)
        {
            PI_AStar<NodeScorer_PathNG_T>::Ptr res = PI_AStar<NodeScorer_PathNG_T>::Create();
            res->Initialize(config);
            return res;
        }
        // Org PI_AStar<NodeExpander_AVT_T,NodeScorer_Goal_T>::Ptr res = PI_AStar<NodeExpander_AVT_T,NodeScorer_Goal_T>::Create();

    }
    if (config.plannerType_ == "TreeDWA")
    {
        if (config.scorerType_ == NodeScorer_Goal_T::NS_NAME)
        {
            PI_Tree<NodeScorer_Goal_T>::Ptr res = PI_Tree<NodeScorer_Goal_T>::Create();
            res->Initialize(config);
            return res;
        }
        if (config.scorerType_ == NodeScorer_Path_T::NS_NAME)
        {
            PI_Tree<NodeScorer_Path_T>::Ptr res = PI_Tree<NodeScorer_Path_T>::Create();
            res->Initialize(config);
            return res;
        }
        if (config.scorerType_ == NodeScorer_PathNG_T::NS_NAME)
        {
            PI_Tree<NodeScorer_PathNG_T>::Ptr res = PI_Tree<NodeScorer_PathNG_T>::Create();
            res->Initialize(config);
            return res;
        }
    }
    if (config.plannerType_ == "DWA")
    {
        if (config.scorerType_ == NodeScorer_Goal_T::NS_NAME)
        {
            PI_DWA<NodeScorer_Goal_T>::Ptr res = PI_DWA<NodeScorer_Goal_T>::Create();
            res->Initialize(config);
            return res;
        }
        if (config.scorerType_ == NodeScorer_Path_T::NS_NAME)
        {
            PI_DWA<NodeScorer_Path_T>::Ptr res = PI_DWA<NodeScorer_Path_T>::Create();
            res->Initialize(config);
            return res;
        }
        if (config.scorerType_ == NodeScorer_PathNG_T::NS_NAME)
        {
            PI_DWA<NodeScorer_PathNG_T>::Ptr res = PI_DWA<NodeScorer_PathNG_T>::Create();
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
