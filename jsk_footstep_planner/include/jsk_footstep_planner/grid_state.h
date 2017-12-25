#ifndef JSK_FOOTSTEP_PLANNER_GRID_STATE_H_
#define JSK_FOOTSTEP_PLANNER_GRID_STATE_H_

namespace jsk_footstep_planner {
  class GridState
  {
  public:
    typedef boost::shared_ptr<GridState> Ptr;
    GridState(int x, int y) : index_x_(x), index_y_(y)
    {
    }
    inline virtual int indexX() { return index_x_; }
    inline virtual int indexY() { return index_y_; }

    inline virtual int getOccupancy() { return 0; }
    inline virtual bool setOccupancy(int occ_) { return false; }
    inline virtual float getCost() { return 0.0; }
    inline virtual bool setCost(float c_) { return false; }
    inline virtual bool isValid() { return true; }
    // for boost unordered_map
    bool operator==(const GridState& other) const
    {
      return ((index_x_ == other.index_x_) &&
              (index_y_ == other.index_y_));
    }
  protected:
    int index_x_;
    int index_y_;
  };

  class OccupancyGridState : public GridState
  {
  public:
    typedef boost::shared_ptr<OccupancyGridState> Ptr;

    OccupancyGridState(int x, int y) : GridState(x, y), occupancy_(0)
    {
    }

    inline virtual int getOccupancy() { return occupancy_; }
    inline virtual bool setOccupancy(int occ_)
    {
      occupancy_ = occ_;
      return true;
    }
    inline virtual bool isValid()
    {
      if(occupancy_ == 0) {
        return true;
      }
      return false;
    }
  protected:
    int occupancy_;
  };

  class CostedGridState : public OccupancyGridState
  {
  public:
    typedef boost::shared_ptr<CostedGridState> Ptr;

    CostedGridState(int x, int y) : OccupancyGridState(x, y), cost_(0.0)
    {
    }

    inline virtual float getCost()
    {
      return cost_;
    }
    inline virtual bool setCost(float c_)
    {
      cost_ = c_;
      return true;
    }
    inline virtual bool isValid() {
      // TODO update for using cost
      if(occupancy_ == 0) {
        return true;
      }
      return false;
    }
  protected:
    float cost_;
  };

  // for boost unordered_map
  inline size_t hash_value(const GridState::Ptr& s)
  {
    return (std::abs(s->indexX() + 32000) << 16) + std::abs(s->indexY() + 32000);
  }
}
#endif
