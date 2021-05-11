#ifndef JSK_FOOTSTEP_PLANNER_GRID_ASTAR_SOLVER_H_
#define JSK_FOOTSTEP_PLANNER_GRID_ASTAR_SOLVER_H_

#include "jsk_footstep_planner/astar_solver.h"
#include <boost/unordered_map.hpp>
#include <cfloat> // DBL_MAX

namespace jsk_footstep_planner
{
  template <class GraphT>
  class GridAStarSolver: public AStarSolver<GraphT>
  {
  public:
    typedef boost::shared_ptr<GridAStarSolver> Ptr;
    typedef typename GraphT::StateT State;
    typedef typename GraphT::StateT::Ptr StatePtr;
    typedef typename GraphT::Ptr GraphPtr;
    typedef typename SolverNode<State, GraphT>::Ptr SolverNodePtr;
    typedef std::vector<typename SolverNode<State, GraphT>::Ptr> Path;
    typedef boost::unordered_map< StatePtr, SolverNodePtr > SolveList;

    GridAStarSolver(GraphPtr graph): AStarSolver<GraphT>(graph) {}

    virtual
    std::vector<typename SolverNode<State, GraphT>::Ptr>
    solve(const ros::WallDuration& timeout = ros::WallDuration(1000000000.0))
    {
      ros::WallTime start_time = ros::WallTime::now();
      SolverNodePtr start_state(new SolverNode<State, GraphT>
                                (graph_->getStartState(),
                                 0, graph_));
      addToOpenList(start_state);
      if (verbose_) { std::cerr << "start" << std::endl; }
      //while (!isOpenListEmpty() && isOK(start_time, timeout)) {
      while (!isOpenListEmpty()) {
        SolverNodePtr target_node = popFromOpenList();
        StatePtr p = target_node->getState();
        if (verbose_) { std::cerr << "t " << p->indexX() << " - " << p->indexY(); }
        if (graph_->isGoal(target_node->getState())) {
          if (verbose_) { std::cerr << " -> g" << std::endl; }
          std::vector<SolverNodePtr> result_path = target_node->getPathWithoutThis();
          result_path.push_back(target_node);
          return result_path;
        } else {
          std::vector<SolverNodePtr> nnodes = target_node->expand(target_node, verbose_);
          if (verbose_) { std::cerr << " -> c "; }
          addToCloseList(target_node);
          for(int i = 0; i < nnodes.size(); i++) {
            SolverNodePtr anode = nnodes[i];
            double prev_cost = 0;
            if(verbose_) {
              std::cerr << "a-" << i << " " << anode->getState()->indexX();
              std::cerr << " - " << anode->getState()->indexY();
            }
            if(findInOpenList(anode, prev_cost)) {
              if(verbose_) { std::cerr << " -o"; }
              if(anode->getCost() < prev_cost) {
                if(verbose_) { std::cerr << "-r"; }
                removeFromOpenList(anode); // replace
                addToOpenList(anode);
              }
            } else if(findInCloseList(anode, prev_cost)) {
              if(verbose_) { std::cerr << " -c"; }
              if(anode->getCost() < prev_cost) {
                if(verbose_) { std::cerr << "-r"; }
                removeFromCloseList(anode);
                addToOpenList(anode);
              }
            } else {
              if(verbose_) { std::cerr << " -n"; }
              addToOpenList(anode);
            }
            if (verbose_) {
              if (i != nnodes.size() -1) {
                std::cerr << " / ";
              }
            }
          }
        }
        if (verbose_) { std::cerr << std::endl; }
      }
      // Failed to search
      return std::vector<SolverNodePtr>();
    }

    ////
    virtual bool isOpenListEmpty()
    {
      return open_list_map_.empty();
    }
    virtual SolverNodePtr popFromOpenList()
    {
      // slow implimentation??
      double min_cost = DBL_MAX;
      SolverNodePtr ret;
      for (typename SolveList::const_iterator it = open_list_map_.begin();
           it != open_list_map_.end(); it++) {
        if (it->second->getSortValue() < min_cost) {
          ret = it->second;
          min_cost = it->second->getSortValue();
        }
      }
      if (!!ret) {
        typename SolveList::const_iterator it
          = open_list_map_.find(ret->getState());
        open_list_map_.erase(it);
        // remove from sorted_list(pop)
      }
      return ret;
    }
    virtual void addToOpenList(SolverNodePtr node)
    {
      typename SolveList::const_iterator it
        = open_list_map_.find(node->getState());
      if(it != open_list_map_.end()) {
        std::cerr << ";;;; warn (duplicate adding to openlist) ;;;;" << std::endl;
      }
      node->setSortValue(AStarSolver<GraphT>::fn(node));
      open_list_map_
        .insert(typename SolveList::value_type(node->getState(), node));
      // add to sorted_list
    }
    virtual bool findInOpenList(SolverNodePtr node, double &cost)
    {
      typename SolveList::const_iterator it
        = open_list_map_.find(node->getState());
      if (it != open_list_map_.end()) {
        cost = it->second->getCost();
        return true;
      }
      return false;
    }
    virtual bool removeFromOpenList(SolverNodePtr node)
    {
      typename SolveList::const_iterator it
        = open_list_map_.find(node->getState());
      if(it != open_list_map_.end()) {
        open_list_map_.erase(it);
        // remove from sorted_list(reconstruct)
        return true;
      } else {
        std::cerr << ";;;; warn (fail remove) ;;;;" << std::endl;
      }
      return false;
    }
    virtual void addToCloseList(SolverNodePtr node)
    {
      // check
      if(AStarSolver<GraphT>::findInCloseList(node->getState())) {
        std::cerr << ";;;; warn (duplicate adding to closelist) ;;;;" << std::endl;
      }
      //
      AStarSolver<GraphT>::addToCloseList(node->getState(), node->getCost());
    }
    virtual bool findInCloseList(SolverNodePtr node, double &cost)
    {
      return AStarSolver<GraphT>::findInCloseList(node->getState(), cost);
    }
    virtual bool removeFromCloseList(SolverNodePtr node)
    {
      return AStarSolver<GraphT>::removeFromCloseList(node->getState());
    }
    ////
    virtual bool getOpenList(std::vector<StatePtr> &lst, std::vector<float> &cost)
    {
      return false;
    }
    virtual bool getCloseList(std::vector<StatePtr> &lst, std::vector<float> &cost)
    {
      return false;
    }

  protected:
    SolveList open_list_map_;
    using Solver<GraphT>::graph_;
    using Solver<GraphT>::verbose_;
  private:

  };
}
#endif
