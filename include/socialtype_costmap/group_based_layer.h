#ifndef AGE_BASED_COSTMAP_LAYER_H_
#define AGE_BASED_COSTMAP_LAYER_H_

#include <ros/ros.h>
#include <socialtype_costmap/group_layer.h>
#include <dynamic_reconfigure/server.h>
#include <socialtype_costmap/GroupBasedLayerConfig.h>

double Gaussian2D(double x, double y, double x0, double y0, double A, double varx, double vary);

namespace socialtype_costmap
{
class GroupBasedLayer : public GroupLayer
{
public:
  GroupBasedLayer()
  {
    layered_costmap_ = NULL;
  }

  virtual void onInitialize();
  std::vector<std::vector<hri_msgs::Person>> createGroups();
  virtual void updateBoundsFromPeople(double* min_x, double* min_y, double* max_x, double* max_y);
  virtual void updateCosts(costmap_2d::Costmap2D& master_grid, int min_i, int min_j, int max_i, int max_j);
  virtual void updateCostForGroup(const std::vector<hri_msgs::Person>& group, costmap_2d::Costmap2D& costmap, double res, int min_i, int min_j, int max_i, int max_j);

protected:
  void configure(GroupBasedLayerConfig &config, uint32_t level);
  double cutoff_, amplitude_, covar_, radius_, factor_, group_factor_;
  dynamic_reconfigure::Server<GroupBasedLayerConfig>* server_;
  dynamic_reconfigure::Server<GroupBasedLayerConfig>::CallbackType f_;
};
};
#endif  // AGE_BASED_COSTMAP_LAYER_H_
