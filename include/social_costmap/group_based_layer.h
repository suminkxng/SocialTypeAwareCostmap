#ifndef AGE_BASED_COSTMAP_LAYER_H_
#define AGE_BASED_COSTMAP_LAYER_H_

#include <ros/ros.h>
#include <social_costmap/group_layer.h>
#include <dynamic_reconfigure/server.h>
#include <social_costmap/GroupBasedLayerConfig.h>

double Gaussian2D(double x, double y, double x0, double y0, double A, double varx, double vary);

namespace social_costmap
{
class GroupBasedLayer : public GroupLayer
{
public:
  GroupBasedLayer()
  {
    layered_costmap_ = NULL;
  }

  virtual void onInitialize();
  virtual void updateBoundsFromPeople(double* min_x, double* min_y, double* max_x, double* max_y);
  virtual void updateCosts(costmap_2d::Costmap2D& master_grid, int min_i, int min_j, int max_i, int max_j);

protected:
  void configure(GroupBasedLayerConfig &config, uint32_t level);
  double amplitude_, group_radius_;
  dynamic_reconfigure::Server<GroupBasedLayerConfig>* server_;
  dynamic_reconfigure::Server<GroupBasedLayerConfig>::CallbackType f_;
};
};
#endif  // AGE_BASED_COSTMAP_LAYER_H_
