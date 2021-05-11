#ifndef JSK_FOOTSTEP_PLANNER_GRID_PERCEPTION_H_
#define JSK_FOOTSTEP_PLANNER_GRID_PERCEPTION_H_

#include <pcl/pcl_base.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/point_representation.h>

#include "jsk_footstep_planner/grid_astar_solver.h"
#include "jsk_footstep_planner/grid_graph.h"

namespace jsk_footstep_planner {
#if 0
  class PerceptionGridState : public CostedGridState
  {
  public:
    typedef boost::shared_ptr<PerceptionGridState> Ptr;

    PerceptionGridState(int x, int y) : CostedGridState(x, y), updated_(false)
    {
    }
    inline virtual bool isValid() {
      // TODO update for using cost
      if(occupancy_ == 0) {
        return true;
      }
      return false;
    }
  protected:
    bool updated_;
  };
#endif
  class PerceptionGridMap : public GridMap <CostedGridState>
  {
  public:
    typedef boost::shared_ptr< PerceptionGridMap > Ptr;
    typedef boost::function<bool(GridState::Ptr)> UpdateCostFunction;

    PerceptionGridMap(int _x, int _y) : GridMap()
    {
      size_x_ = _x;
      size_y_ = _y;
      resetGrid();
    }

    inline virtual bool checkState(int ix, int iy)
    {
      StatePtr pt = state_list_.at(index(ix, iy));
      if(!pt) {
        return false;
      }
      return true;
    }

    inline virtual StatePtr getState(int ix, int iy)
    {
      StatePtr pt = state_list_.at(index(ix, iy));
      if(!pt) {
        StatePtr new_pt(new MapState(ix, iy));
        state_list_[index(ix, iy)] = new_pt;
        updateCost(new_pt);
        return new_pt;
      }
      return pt;
    }

    virtual bool updateCost(StatePtr st)
    {
#if 0
      // update around the state
      int ix = st->indexX();
      int iy = st->indexY();
      for(int y = -1; y < 2; y++) {
        for(int x = -1; x < 2; x++) {
          if(inRange(ix + x, iy + y)) {
            StatePtr pt = state_list_.at(index(ix+x, iy+y));
            if(!pt) {
              StatePtr new_pt(new MapState(ix+x, iy+y));
              state_list_[index(ix, iy)] = new_pt;
              cost_func_(new_pt);
            }
          }
        }
      }
#endif
      return cost_func_(st);
    }

    virtual void setCostFunction(UpdateCostFunction func)
    {
      cost_func_ = func;
    }

  protected:
    virtual void resetGrid() {
      state_list_.clear();
      state_list_.resize(size_x_ * size_y_);
    }

    UpdateCostFunction cost_func_;
  };

  typedef GridGraph<CostedGridState> PerceptionGridGraph;

  // default heuristic function
  double gridPerceptionHeuristicDistance(SolverNode<PerceptionGridGraph::State, PerceptionGridGraph>::Ptr node,
                                         PerceptionGridGraph::Ptr graph) {

    int ix = node->getState()->indexX();
    int iy = node->getState()->indexY();
    double gx = graph->getGoalState()->indexX() - ix;
    double gy = graph->getGoalState()->indexY() - iy;

    return std::sqrt(gx * gx + gy * gy);
  }
}

#endif
