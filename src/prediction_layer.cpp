#include <socialtype_costmap/prediction_layer.h>
#include <algorithm>
#include <pluginlib/class_list_macros.h>
#include <socialtype_costmap/PredictionLayerConfig.h>

PLUGINLIB_EXPORT_CLASS(socialtype_costmap::PredictionLayer, costmap_2d::Layer)

namespace socialtype_costmap
{

PredictionLayer::PredictionLayer() {}

PredictionLayer::~PredictionLayer() {}

void PredictionLayer::onInitialize()
{
    trajectory_sub_ = nh_.subscribe("/predicted_traj", 1, &PredictionLayer::trajectoryCallback, this);
    ros::NodeHandle nh("~/" + name_), g_nh;
    server_ = new dynamic_reconfigure::Server<socialtype_costmap::PredictionLayerConfig>(nh);
    f_ = boost::bind(&PredictionLayer::configure, this, _1, _2);
    server_->setCallback(f_);
}

void PredictionLayer::trajectoryCallback(const hri_msgs::TrajectoryArray::ConstPtr& msg)
{
    traj_positions_.clear();
    for (const auto& traj : msg->trajectories)
    {
        for (const auto& pos : traj.people)
        {
            traj_positions_.push_back(pos);
        }
    }
}

void PredictionLayer::updateBounds(double origin_x, double origin_y, double origin_yaw, double* min_x, double* min_y, double* max_x, double* max_y)
{
    if (!enabled_ || traj_positions_.empty()) {
        return;
    }

    for (const auto& pos : traj_positions_)
    {
        *min_x = std::min(*min_x, static_cast<double>(pos.pose.position.x));
        *min_y = std::min(*min_y, static_cast<double>(pos.pose.position.y));
        *max_x = std::max(*max_x, static_cast<double>(pos.pose.position.x));
        *max_y = std::max(*max_y, static_cast<double>(pos.pose.position.y));
    }
}

void PredictionLayer::updateCosts(costmap_2d::Costmap2D& master_grid, int min_i, int min_j, int max_i, int max_j)
{
    if (!enabled_)
        return;

    unsigned int mx, my;

    for (const auto& pos : traj_positions_)
    {
        if (master_grid.worldToMap(pos.pose.position.x, pos.pose.position.y, mx, my))
        {
            master_grid.setCost(mx, my, costmap_2d::LETHAL_OBSTACLE);  // LETHAL_OBSTACLE은 costmap_2d에서 가장 높은 비용을 나타냅니다.
        }
    }
}


void PredictionLayer::configure(socialtype_costmap::PredictionLayerConfig &config, uint32_t level)
{
    enabled_ = config.enabled;
    // cost_threshold_ = config.cost_threshold; 
}

} // namespace socialtype_costmap
