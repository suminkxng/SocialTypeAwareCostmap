#ifndef HUMAN_COSTMAP_LAYER_H_
#define HUMAN_COSTMAP_LAYER_H_

#include <ros/ros.h>
#include <costmap_2d/layer.h>
#include <costmap_2d/layered_costmap.h>
#include <hri_msgs/PersonArray.h>
#include <boost/thread.hpp>
#include <list>

namespace socialtype_costmap
{
class HumanLayer : public costmap_2d::Layer
{
public:
  HumanLayer() : last_detection_time_(0), detection_delay_(3.0)  // 초기화
  {
    layered_costmap_ = NULL;
  }

  virtual void onInitialize();
  virtual void updateBounds(double origin_x, double origin_y, double origin_yaw, double* min_x, double* min_y,
                            double* max_x, double* max_y);
  virtual void updateCosts(costmap_2d::Costmap2D& master_grid, int min_i, int min_j, int max_i, int max_j) = 0;

  virtual void updateBoundsFromPeople(double* min_x, double* min_y, double* max_x, double* max_y) = 0;

  bool isDiscretized()
  {
    return false;
  }

protected:
  void peopleCallback(const hri_msgs::PersonArray& people);
  ros::Subscriber people_sub_;
  hri_msgs::PersonArray people_list_;
  std::vector<hri_msgs::Person> transformed_people_;
  ros::Duration people_keep_time_;
  boost::recursive_mutex lock_;
  bool first_time_;
  double last_min_x_, last_min_y_, last_max_x_, last_max_y_;
  ros::Time last_detection_time_;
  double detection_delay_;
  hri_msgs::PersonArray last_detected_people_;  

  double gaussian(double x, double y, double x0, double y0, double A, double varx, double vary, double skew) {
      double dx = x - x0, dy = y - y0;
      double h = sqrt(dx * dx + dy * dy);
      double angle = atan2(dy, dx);
      double mx = cos(angle - skew) * h;
      double my = sin(angle - skew) * h;
      double f1 = pow(mx, 2.0) / (2.0 * varx),
            f2 = pow(my, 2.0) / (2.0 * vary);
      return A * exp(-(f1 + f2));
  }

double get_radius(double cutoff, double A, double var) {
      return sqrt(-2 * var * log(cutoff / A));
  }

};
}  // namespace socialtype_costmap

#endif  // HUMAN_COSTMAP_LAYER_H_
