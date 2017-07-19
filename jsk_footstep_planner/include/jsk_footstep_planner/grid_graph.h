#ifndef JSK_FOOTSTEP_PLANNER_GRID_GRAPH_H_
#define JSK_FOOTSTEP_PLANNER_GRID_GRAPH_H_

#include "jsk_footstep_planner/graph.h"
#include "jsk_footstep_planner/grid_state.h"

namespace jsk_footstep_planner
{
  template <class GStateT>
  class GridMap
  {
  public:
    typedef boost::shared_ptr< GridMap > Ptr;
    typedef GridState State;
    typedef GridState::Ptr StatePtr;
    typedef GStateT MapState;
    typedef boost::shared_ptr< GStateT > MapStatePtr;


    GridMap(int _x, int _y) : size_x_(_x), size_y_(_y)
    {
      createGrid();
    }
    inline virtual StatePtr getState(int ix, int iy)
    {
      return state_list_.at(index(ix, iy));
    }
    inline virtual bool setCost(std::vector<float> in)
    {
      size_t cnt = state_list_.size() <= in.size() ? state_list_.size() : in.size();
      for(int i = 0; i < cnt; i++) {
        state_list_[i]->setCost(in[i]);
      }
    }
    inline virtual bool setCost(int ix, int iy, float cost = 0.0)
    {
      return getState(ix, iy)->setCost(cost);
    }
    inline virtual bool setOccupancy(std::vector<int> in)
    {
      size_t cnt = state_list_.size() <= in.size() ? state_list_.size() : in.size();
      for(int i = 0; i < cnt; i++) {
        state_list_[i]->setOccupancy(in[i]);
      }
    }
    inline virtual bool setOccupancy(int ix, int iy, int occupancy = 0)
    {
      return getState(ix, iy)->setOccupancy(occupancy);
    }
    inline virtual int getOccupancy(int ix, int iy)
    {
      return getState(ix, iy)->getOccupancy();
    }
    inline virtual float getCost(int ix, int iy)
    {
      return getState(ix, iy)->getCost();
    }
    inline virtual bool isValid(int ix, int iy)
    {
      return getState(ix, iy)->isValid();
    }

    inline virtual int sizeX() { return size_x_; }
    inline virtual int sizeY() { return size_y_; }
    inline virtual int index(int ix, int iy) { return ((size_x_ * iy) + ix); }
    inline virtual int inRange(int ix, int iy) { return (ix >= 0 && ix < size_x_ && iy >= 0 && iy < size_y_ ); }
  protected:
    GridMap() {}
    virtual void createGrid() {
      state_list_.resize(size_x_ * size_y_);
      for (int y = 0; y < size_y_; y++) {
        for (int x = 0; x < size_x_; x++) {
          StatePtr st(new MapState(x, y));
          state_list_[index(x, y)] = st;
        }
      }
    }
    int size_x_;
    int size_y_;
    std::vector<StatePtr > state_list_;
  };

  template <class GStateT>
  class GridGraph: public Graph<GridState>
  {
  public:
    typedef boost::shared_ptr<GridGraph> Ptr;
    typedef GridMap<GStateT> GridType;
    typedef GStateT MapState;
    typedef typename GStateT::Ptr MapStatePtr;
    typedef Graph<GridState>::StateT State;

    GridGraph(typename GridType::Ptr gr)
    {
      gridmap_ = gr;
    }

    StatePtr getState(int ix, int iy) {
      if(gridmap_->inRange(ix, iy)) {
        return gridmap_->getState(ix, iy);
      }
      StatePtr p;
      p.reset();
      return p;
    }

    virtual std::vector<StatePtr> successors(StatePtr target_state)
    {
      std::vector<StatePtr> ret;
      int x_offset = target_state->indexX();
      int y_offset = target_state->indexY();
      for (int x = -1; x < 2; x++) {
        for (int y = -1; y < 2; y++) {
          if (x != 0 || y != 0) {
            StatePtr st =
              getState(x + x_offset, y + y_offset);
            if (!!st) {
              if(st->isValid()) {
                ret.push_back(st);
              }
            }
          }
        }
      }
      return ret;
    }
    virtual bool isGoal(StatePtr state)
    {
      return (state == goal_state_);
    }

    virtual double pathCost(StatePtr from,
                            StatePtr to, double prev_cost)
    {
      double gx = from->indexX() - to->indexX();
      double gy = from->indexY() - to->indexY();
      return prev_cost + std::sqrt(gx * gx + gy * gy) + to->getCost();
    }

  protected:
    typename GridType::Ptr gridmap_;
  private:
  };

  typedef GridMap<GridState> SimpleGridMap;
  typedef GridMap<OccupancyGridState> OccupancyGridMap;
  typedef GridMap<CostedGridState> CostedGridMap;

  typedef GridGraph<GridState> SimpleGridGraph;
  typedef GridGraph<OccupancyGridState> OccupancyGridGraph;
  typedef GridGraph<CostedGridState> CostedGridGraph;
}

#endif
