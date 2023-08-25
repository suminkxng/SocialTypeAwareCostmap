#ifndef PREDICTION_COSTMAP_LAYER_H_
#define PREDICTION_COSTMAP_LAYER_H_

#include <ros/ros.h>
#include <costmap_2d/layer.h>
#include <costmap_2d/layered_costmap.h>
#include <dynamic_reconfigure/server.h>
#include <hri_msgs/TrajectoryArray.h>
#include <social_costmap/PredictionLayerConfig.h>

namespace social_costmap
{

class PredictionLayer : public costmap_2d::Layer
{
public:
    PredictionLayer();
    virtual ~PredictionLayer();

    virtual void onInitialize();
    virtual void updateBounds(double origin_x, double origin_y, double origin_yaw, double* min_x, double* min_y, double* max_x, double* max_y);
    virtual void updateCosts(costmap_2d::Costmap2D& master_grid, int min_i, int min_j, int max_i, int max_j);

private:
    void trajectoryCallback(const hri_msgs::TrajectoryArray::ConstPtr& msg);
    
    ros::Subscriber trajectory_sub_;
    std::vector<hri_msgs::Person> traj_positions_;

protected:
    void configure(social_costmap::PredictionLayerConfig &config, uint32_t level);
    ros::NodeHandle nh_;
    int cost_threshold_;
    bool enabled_;
    dynamic_reconfigure::Server<social_costmap::PredictionLayerConfig>* server_;
    dynamic_reconfigure::Server<social_costmap::PredictionLayerConfig>::CallbackType f_;
};

} // namespace social_costmap

#endif
