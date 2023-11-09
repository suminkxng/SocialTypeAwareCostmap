#include <social_costmap/group_based_layer.h>
#include <math.h>
#include <angles/angles.h> 
#include <pluginlib/class_list_macros.h> 
#include <algorithm>
#include <list>

PLUGINLIB_EXPORT_CLASS(social_costmap::GroupBasedLayer, costmap_2d::Layer)

using costmap_2d::NO_INFORMATION;
using costmap_2d::LETHAL_OBSTACLE;
using costmap_2d::FREE_SPACE;


geometry_msgs::Point computeGroupCenter(const std::vector<hri_msgs::Person>& group) {
  geometry_msgs::Point center;
  center.x = 0;
  center.y = 0;
  for(const auto& person : group) {
    center.x += person.pose.position.x;
    center.y += person.pose.position.y;
  }
  center.x /= group.size();
  center.y /= group.size();
  return center;
}

double computeGroupSize(const std::vector<hri_msgs::Person>& group, const geometry_msgs::Point& center) {
  double max_distance = 0;
  for(const auto& person : group) {
    double distance = sqrt(pow(person.pose.position.x - center.x, 2) + pow(person.pose.position.y - center.y, 2));
    // ROS_INFO("Person position: (%f, %f), Group center: (%f, %f), Distance: %f", person.pose.position.x, person.pose.position.y, center.x, center.y, distance);
    max_distance = std::max(max_distance, distance);
  }

  return max_distance;
}

geometry_msgs::Vector3 computeGroupVelocityAndDirection(const std::vector<hri_msgs::Person>& group) {
    geometry_msgs::Vector3 avg_velocity;
    avg_velocity.x = 0;
    avg_velocity.y = 0;
    avg_velocity.z = 0; // For completeness, but we won't be using z.

    for(const auto& person : group) {
        avg_velocity.x += person.velocity.x;
        avg_velocity.y += person.velocity.y;
    }

    avg_velocity.x /= group.size();
    avg_velocity.y /= group.size();
    return avg_velocity;
}

namespace social_costmap
{
void GroupBasedLayer::onInitialize()
{
  HumanLayer::onInitialize();
  ros::NodeHandle nh("~/" + name_), g_nh;
  server_ = new dynamic_reconfigure::Server<GroupBasedLayerConfig>(nh);
  f_ = boost::bind(&GroupBasedLayer::configure, this, _1, _2);
  server_->setCallback(f_);
}

std::vector<std::vector<hri_msgs::Person>> GroupBasedLayer::createGroups() {
    std::vector<std::vector<hri_msgs::Person>> groups;
    std::vector<hri_msgs::Person> group;
    for (size_t i = 0; i < transformed_people_.size(); ++i) {
        hri_msgs::Person person = transformed_people_[i];
        if (people_list_.people[i].type == 3) {
            group.push_back(person);
        }
        
        if (people_list_.people[i].type != 3 || i == transformed_people_.size() - 1) {
            if (!group.empty()) {
                groups.push_back(group);
                group.clear();
            }
        }
    }
    return groups;
}

void GroupBasedLayer::updateBoundsFromPeople(double* min_x, double* min_y, double* max_x, double* max_y) {
    // ROS_INFO("Inside updateBoundsFromPeople");
    auto groups = createGroups();

    for(const auto& group : groups) {
        geometry_msgs::Point group_center = computeGroupCenter(group);
        geometry_msgs::Vector3 group_velocity = computeGroupVelocityAndDirection(group);
        double mag = sqrt(pow(group_velocity.x, 2) + pow(group_velocity.y, 2));
        double factor = 1.0 + mag * factor_;

        double covar = covar_;
        auto group_size = computeGroupSize(group, group_center);
        double point = get_radius(cutoff_, amplitude_, covar_ * factor * group_size * group_factor_);

        *min_x = std::min(*min_x, group_center.x - point);
        *min_y = std::min(*min_y, group_center.y - point);
        *max_x = std::max(*max_x, group_center.x + point);
        *max_y = std::max(*max_y, group_center.y + point);
    }
}

void GroupBasedLayer::updateCostForGroup(const std::vector<hri_msgs::Person>& group, costmap_2d::Costmap2D& costmap, double res, int min_i, int min_j, int max_i, int max_j)
{
    // ROS_INFO("updateCostForGroup called");
    geometry_msgs::Point group_center = computeGroupCenter(group);
    auto group_size = computeGroupSize(group, group_center);
    geometry_msgs::Vector3 group_velocity = computeGroupVelocityAndDirection(group);
    double angle = atan2(group_velocity.y, group_velocity.x);

    double mag = sqrt(pow(group_velocity.x, 2) + pow(group_velocity.y, 2));
    double factor = 1.0 + mag * factor_;
    double covar = covar_;

    double base = get_radius(cutoff_, amplitude_, covar);
    double point = get_radius(cutoff_, amplitude_, covar * factor * group_size * group_factor_);
    unsigned int width = std::max(1, static_cast<int>((base + point) / res)),
                 height = std::max(1, static_cast<int>((base + point) / res));

    // Use group center as the center for Gaussian
    double cx = group_center.x, cy = group_center.y;
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
    costmap.worldToMapNoBounds(ox, oy, dx, dy);

    int start_x = 0, start_y = 0, end_x = width, end_y = height;
    if (dx < 0)
      start_x = -dx;
    else if (dx + width > costmap.getSizeInCellsX())
      end_x = std::max(0, static_cast<int>(costmap.getSizeInCellsX()) - dx);

    if (static_cast<int>(start_x + dx) < min_i)
      start_x = min_i - dx;
    if (static_cast<int>(end_x + dx) > max_i)
      end_x = max_i - dx;

    if (dy < 0)
      start_y = -dy;
    else if (dy + height > costmap.getSizeInCellsY())
      end_y = std::max(0, static_cast<int>(costmap.getSizeInCellsY()) - dy);

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
        unsigned char old_cost = costmap.getCost(i + dx, j + dy);
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
        costmap.setCost(i + dx, j + dy, std::max(cvalue, old_cost));
      }
    }
  }

void GroupBasedLayer::updateCosts(costmap_2d::Costmap2D& master_grid, int min_i, int min_j, int max_i, int max_j) {
    // ROS_INFO("Inside updateCosts");
    boost::recursive_mutex::scoped_lock lock(lock_);
    if (!enabled_) return;
    if (people_list_.people.empty()) return;

    auto groups = createGroups();
    // ROS_INFO("Number of groups: %zu", groups.size());

    costmap_2d::Costmap2D* costmap = layered_costmap_->getCostmap();
    double res = costmap->getResolution();
    
    for (const auto& group : groups) {
        updateCostForGroup(group, *costmap, res, min_i, min_j, max_i, max_j);
    }
}

void GroupBasedLayer::configure(GroupBasedLayerConfig &config, uint32_t level)
{
  covar_ = config.covariance;
  amplitude_ = config.amplitude;
  cutoff_ = config.cutoff;
  factor_ = config.factor;
  group_factor_ = config.group_size_factor;
  enabled_ = config.enabled;
}
};  // namespace social_costmap
