#include <social_costmap/age_based_layer.h>
#include <math.h>
#include <angles/angles.h>
#include <pluginlib/class_list_macros.h>
#include <algorithm>
#include <list>
#include <tf/transform_datatypes.h>

PLUGINLIB_EXPORT_CLASS(social_costmap::AgeBasedLayer, costmap_2d::Layer)

using costmap_2d::NO_INFORMATION;
using costmap_2d::LETHAL_OBSTACLE;
using costmap_2d::FREE_SPACE;


namespace social_costmap
{
void AgeBasedLayer::onInitialize()
{
  HumanLayer::onInitialize();
  ros::NodeHandle nh("~/" + name_), g_nh;
  server_ = new dynamic_reconfigure::Server<AgeBasedLayerConfig>(nh);
  f_ = boost::bind(&AgeBasedLayer::configure, this, _1, _2);
  server_->setCallback(f_);
}

void AgeBasedLayer::updateBoundsFromPeople(double* min_x, double* min_y, double* max_x, double* max_y)
{
  // std::vector<hri_msgs::Person>::iterator p_it;

  for (size_t i = 0; i < transformed_people_.size(); ++i)
  {
    hri_msgs::Person person = transformed_people_[i];

    double mag = sqrt(pow(person.velocity.x, 2) + pow(person.velocity.y, 2));
    double factor = 1.0 + mag * factor_;

    double covar = covar_;
    
    if (people_list_.people[i].type == 0){
      covar = adult_covar_;
    }
    else if (people_list_.people[i].type == 1){
      covar = child_covar_;
    }
    else if (people_list_.people[i].type == 3)
      return;

    double point = get_radius(cutoff_, amplitude_, covar * factor);

    *min_x = std::min(*min_x, person.pose.position.x - point);
    *min_y = std::min(*min_y, person.pose.position.y - point);
    *max_x = std::max(*max_x, person.pose.position.x + point);
    *max_y = std::max(*max_y, person.pose.position.y + point);
  }
}

void AgeBasedLayer::updateCosts(costmap_2d::Costmap2D& master_grid, int min_i, int min_j, int max_i, int max_j)
{
  boost::recursive_mutex::scoped_lock lock(lock_);
  if (!enabled_) return;

  if (people_list_.people.size() == 0)
    return;
  if (cutoff_ >= amplitude_)
    return;

  // std::vector<hri_msgs::Person>::iterator p_it;
  costmap_2d::Costmap2D* costmap = layered_costmap_->getCostmap();
  double res = costmap->getResolution();

  for (size_t i = 0; i < transformed_people_.size(); ++i)
  {
    hri_msgs::Person person = transformed_people_[i];
    double angle = atan2(person.velocity.y, person.velocity.x);
    double mag = sqrt(pow(person.velocity.x, 2) + pow(person.velocity.y, 2));
    double factor = 1.0 + mag * factor_;
    double covar = covar_;
    
    if (people_list_.people[i].type == 0){
      covar = adult_covar_;
    }
    else if (people_list_.people[i].type == 1){
      covar = child_covar_;
    }
    else if (people_list_.people[i].type == 3)
      return;

    double base = get_radius(cutoff_, amplitude_, covar);
    double point = get_radius(cutoff_, amplitude_, covar * factor);

    unsigned int width = std::max(1, static_cast<int>((base + point) / res)),
                 height = std::max(1, static_cast<int>((base + point) / res));

    double cx = person.pose.position.x, cy = person.pose.position.y;

    double ox, oy;
    if (sin(angle) > 0)
      oy = cy - base;
    else
      oy = cy + (point - base) * sin(angle) - base;

    if (cos(angle) >= 0)
      ox = cx - base;
    else
      ox = cx + (point - base) * cos(angle) - base;


    int dx, dy;
    costmap->worldToMapNoBounds(ox, oy, dx, dy);

    int start_x = 0, start_y = 0, end_x = width, end_y = height;
    if (dx < 0)
      start_x = -dx;
    else if (dx + width > costmap->getSizeInCellsX())
      end_x = std::max(0, static_cast<int>(costmap->getSizeInCellsX()) - dx);

    if (static_cast<int>(start_x + dx) < min_i)
      start_x = min_i - dx;
    if (static_cast<int>(end_x + dx) > max_i)
      end_x = max_i - dx;

    if (dy < 0)
      start_y = -dy;
    else if (dy + height > costmap->getSizeInCellsY())
      end_y = std::max(0, static_cast<int>(costmap->getSizeInCellsY()) - dy);

    if (static_cast<int>(start_y + dy) < min_j)
      start_y = min_j - dy;
    if (static_cast<int>(end_y + dy) > max_j)
      end_y = max_j - dy;

    double bx = ox + res / 2,
           by = oy + res / 2;
    for (int i = start_x; i < end_x; i++)
    {
      for (int j = start_y; j < end_y; j++)
      {
        unsigned char old_cost = costmap->getCost(i + dx, j + dy);
        if (old_cost == costmap_2d::NO_INFORMATION)
          continue;

        double x = bx + i * res, y = by + j * res;
        double ma = atan2(y - cy, x - cx);
        double diff = angles::shortest_angular_distance(angle, ma);
        double a;
        if (fabs(diff) < M_PI / 2)
          a = gaussian(x, y, cx, cy, amplitude_, covar * factor, covar, angle);
        else
          a = gaussian(x, y, cx, cy, amplitude_, covar,       covar, 0);

        if (a < cutoff_)
          continue;
        unsigned char cvalue = (unsigned char) a;
        costmap->setCost(i + dx, j + dy, std::max(cvalue, old_cost));
      }
    }
  }
}

void AgeBasedLayer::configure(AgeBasedLayerConfig &config, uint32_t level)
{
  covar_ = config.covariance;
  adult_covar_ = config.adult_covar;
  child_covar_ = config.child_covar;
  amplitude_ = config.amplitude;
  cutoff_ = config.cutoff;
  factor_ = config.factor;
  enabled_ = config.enabled;
}
};  // namespace social_costmap
