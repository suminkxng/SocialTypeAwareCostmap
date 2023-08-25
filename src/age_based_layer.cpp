#include <social_costmap/age_based_layer.h>
#include <math.h>
#include <angles/angles.h> // 각도 관련 연산을 위한 라이브러리
#include <pluginlib/class_list_macros.h> // ROS 플러그인을 동적으로 로드하기 위한 매크로
#include <algorithm>
#include <list>

PLUGINLIB_EXPORT_CLASS(social_costmap::AgeBasedLayer, costmap_2d::Layer) // 클래스를 플러그인으로서 노출

using costmap_2d::NO_INFORMATION;
using costmap_2d::LETHAL_OBSTACLE;
using costmap_2d::FREE_SPACE;

// double Guassian1D(double x, double x0, double A, double varx){
// double dx = x-x0;
// return A*exp(-pow(dx,2.0)/(2.0*varx));
// }

// double Gaussian2D(double x, double y, double x0, double y0, double A, double varx, double vary)
// {
// double dx = x - x0, dy = y - y0;
// double d = sqrt(dx * dx + dy * dy);
// double theta = atan2(dy, dx);
// double X = d*cos(theta), Y = d*sin(theta);
// return A/std::max(d,1.0) * Guassian1D(X,0.0,1.0,varx) * Guassian1D(Y,0.0,1.0,vary);
// }

double Gaussian2D(double x, double y, double x0, double y0, double A, double varx, double vary) {
    double dx = x - x0;
    double dy = y - y0;
    double f1 = pow(dx, 2.0) / (2.0 * varx);
    double f2 = pow(dy, 2.0) / (2.0 * vary);
    return A * exp(-(f1 + f2));
}


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
  std::vector<hri_msgs::Person>::iterator p_it;

    for (size_t i = 0; i < transformed_people_.size(); ++i)
    {
        hri_msgs::Person person = transformed_people_[i];
        double radius = adult_radius_;
        if (people_list_.people[i].id == 0){
            radius = adult_radius_;
        }
        else if (people_list_.people[i].id == 1){
            radius = child_radius_;
        }

        *min_x = std::min(*min_x, person.pose.position.x - radius);
        *min_y = std::min(*min_y, person.pose.position.y - radius);
        *max_x = std::max(*max_x, person.pose.position.x + radius);
        *max_y = std::max(*max_y, person.pose.position.y + radius);
    }
}

void AgeBasedLayer::updateCosts(costmap_2d::Costmap2D& master_grid, int min_i, int min_j, int max_i, int max_j)
{
  boost::recursive_mutex::scoped_lock lock(lock_);
  if (!enabled_) return;

  if (people_list_.people.size() == 0)
    return;

  std::vector<hri_msgs::Person>::iterator p_it;

  costmap_2d::Costmap2D* costmap = layered_costmap_->getCostmap();
  double res = costmap->getResolution();

  for (size_t i = 0; i < transformed_people_.size(); ++i)
  {
    hri_msgs::Person person = transformed_people_[i];
    double rad = adult_radius_;
    if (people_list_.people[i].id == 0){
        rad = adult_radius_;
    }
    else if (people_list_.people[i].id == 1){
        rad = child_radius_;
    }

    unsigned int width = std::max(1, static_cast<int>((3*rad) / res)),
                 height = std::max(1, static_cast<int>((3*rad) / res));

    double cx = person.pose.position.x, cy = person.pose.position.y;
    double ox = cx - rad, oy = cy - rad;

    int mx, my;
    costmap->worldToMapNoBounds(ox, oy, mx, my);

    int start_x = 0, start_y = 0, end_x = width, end_y = height;
    if (mx < 0)
      start_x = -mx;
    else if (mx + width > costmap->getSizeInCellsX())
      end_x = std::max(0, static_cast<int>(costmap->getSizeInCellsX()) - mx);

    if (static_cast<int>(start_x + mx) < min_i)
      start_x = min_i - mx;
    if (static_cast<int>(end_x + mx) > max_i)
      end_x = max_i - mx;

    if (my < 0)
      start_y = -my;
    else if (my + height > costmap->getSizeInCellsY())
      end_y = std::max(0, static_cast<int>(costmap->getSizeInCellsY()) - my);

    if (static_cast<int>(start_y + my) < min_j)
      start_y = min_j - my;
    if (static_cast<int>(end_y + my) > max_j)
      end_y = max_j - my;

    double bx = ox + res / 2,
           by = oy + res / 2;

    double var = rad;

    for (int i = start_x; i < end_x; i++)
    {
      for (int j = start_y; j < end_y; j++)
      {
        unsigned char old_cost = costmap->getCost(i + mx, j + my);
        if (old_cost == costmap_2d::NO_INFORMATION)
          continue;

        double x = bx + i * res, y = by + j * res;
        double val;
        val = Gaussian2D(x, y, cx, cy, amplitude_, var, var);
        double rad_actual = sqrt(-2*var*log(val/amplitude_));

        if (rad_actual > rad)
          continue;

        unsigned char cvalue = (unsigned char) val;
        costmap->setCost(i + mx, j + my, std::max(cvalue, old_cost));
      }
    }
  }
}


void AgeBasedLayer::configure(AgeBasedLayerConfig &config, uint32_t level)
{
  amplitude_ = config.amplitude;
  adult_radius_ = config.adult_radius;
  child_radius_ = config.child_radius;
  enabled_ = config.enabled;
}
};  // namespace social_costmap
