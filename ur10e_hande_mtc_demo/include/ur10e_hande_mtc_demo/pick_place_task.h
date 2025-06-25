#pragma once

#include <moveit/planning_scene/planning_scene.h>
#include <moveit/task_constructor/task.h>
#include <moveit/task_constructor/solvers.h>
#include <moveit/task_constructor/stages.h>
#include <moveit/robot_model_loader/robot_model_loader.h>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

namespace ur10e_hande_mtc_demo {

using namespace moveit::task_constructor;

class PickPlaceTask {
public:
    PickPlaceTask(const std::string& task_name);
    ~PickPlaceTask() = default;

    bool init(rclcpp::Node::SharedPtr node);
    bool plan(size_t max_solutions);
    bool execute();

private:
    void setupDemoScene();
    void loadTask();
    void setupPlanningPipeline();

    Task task_;
    rclcpp::Node::SharedPtr node_;
    
    // Robot configuration
    std::string arm_group_name_;
    std::string eef_name_;
    std::string hand_group_name_;
    std::string hand_frame_;
    
    // Task objects
    std::string object_name_;
    std::string object_reference_frame_;
    std::string surface_link_;
    std::vector<double> object_dimensions_;
    
    // Poses
    geometry_msgs::msg::PoseStamped object_pose_;
    geometry_msgs::msg::PoseStamped place_pose_;
    std::vector<double> table_dimensions_;
    geometry_msgs::msg::PoseStamped table_pose_;
    
    // Planning
    std::shared_ptr<solvers::PipelinePlanner> sampling_planner_;
    std::shared_ptr<solvers::CartesianPath> cartesian_planner_;
};

void setupDemoScene(rclcpp::Node::SharedPtr node);

}  // namespace ur10e_hande_mtc_demo