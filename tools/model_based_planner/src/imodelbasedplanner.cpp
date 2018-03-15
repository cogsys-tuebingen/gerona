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

    if (config.plannerType_ == "AStar_AngularVel_Goal_WSPL")
    {
        // Org PI_AStar<NodeExpander_AVT_T,NodeScorer_Goal_T>::Ptr res = PI_AStar<NodeExpander_AVT_T,NodeScorer_Goal_T>::Create();
        PI_AStar<NodeExpander_AVNI_T,NodeScorer_Goal_T>::Ptr res = PI_AStar<NodeExpander_AVNI_T,NodeScorer_Goal_T>::Create();
        res->Initialize(config);
        return res;
    }
    if (config.plannerType_ == "TreeDWA_AngularVel_Goal_WSPL")
    {
        PI_Tree<NodeExpander_AVNI_T,NodeScorer_Goal_T>::Ptr res = PI_Tree<NodeExpander_AVNI_T,NodeScorer_Goal_T>::Create();
        res->Initialize(config);
        return res;
    }
    if (config.plannerType_ == "DWA_AngularVel_Goal_WSPL")
    {
        PI_DWA<NodeExpander_AVNI_T,NodeScorer_Goal_T>::Ptr res = PI_DWA<NodeExpander_AVNI_T,NodeScorer_Goal_T>::Create();
        res->Initialize(config);
        return res;
    }

    if (config.plannerType_ == "AStar_LinAngVel_Goal_WSPL")
    {
        // Org PI_AStar<NodeExpander_AVT_T,NodeScorer_Goal_T>::Ptr res = PI_AStar<NodeExpander_AVT_T,NodeScorer_Goal_T>::Create();
        PI_AStar<NodeExpander_LAVT_T,NodeScorer_Goal_T>::Ptr res = PI_AStar<NodeExpander_LAVT_T,NodeScorer_Goal_T>::Create();
        res->Initialize(config);
        return res;
    }
    if (config.plannerType_ == "TreeDWA_LinAngVel_Goal_WSPL")
    {
        PI_Tree<NodeExpander_LAVT_T,NodeScorer_Goal_T>::Ptr res = PI_Tree<NodeExpander_LAVT_T,NodeScorer_Goal_T>::Create();
        res->Initialize(config);
        return res;
    }
    if (config.plannerType_ == "DWA_LinAngVel_Goal_WSPL")
    {
        PI_DWA<NodeExpander_LAVT_T,NodeScorer_Goal_T>::Ptr res = PI_DWA<NodeExpander_LAVT_T,NodeScorer_Goal_T>::Create();
        res->Initialize(config);
        return res;
    }


    return nullptr;
}
