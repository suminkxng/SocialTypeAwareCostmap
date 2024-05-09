#ifndef AGE_BASED_COSTMAP_LAYER_H_
#define AGE_BASED_COSTMAP_LAYER_H_

#include <ros/ros.h>
#include <socialtype_costmap/human_layer.h>
#include <dynamic_reconfigure/server.h>
#include <socialtype_costmap/AgeBasedLayerConfig.h>

double Gaussian2D(double x, double y, double x0, double y0, double A, double varx, double vary);

namespace socialtype_costmap
{
class AgeBasedLayer : public HumanLayer
{
public:
  AgeBasedLayer()
  {
    layered_costmap_ = NULL;
  }

  virtual void onInitialize();
  virtual void updateBoundsFromPeople(double* min_x, double* min_y, double* max_x, double* max_y);
  virtual void updateCosts(costmap_2d::Costmap2D& master_grid, int min_i, int min_j, int max_i, int max_j);

protected:
  void configure(AgeBasedLayerConfig &config, uint32_t level);
  double cutoff_, amplitude_, covar_, radius_, adult_covar_, child_covar_, factor_;
  dynamic_reconfigure::Server<AgeBasedLayerConfig>* server_;
  dynamic_reconfigure::Server<AgeBasedLayerConfig>::CallbackType f_;
};
};
#endif  // AGE_BASED_COSTMAP_LAYER_H_
