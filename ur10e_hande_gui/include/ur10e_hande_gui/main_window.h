#ifndef MAIN_WINDOW_H
#define MAIN_WINDOW_H

#include <QMainWindow>
#include <QLabel>
#include <QPushButton>
#include <QTimer>
#include <QLineEdit> // Added for QLineEdit

#include "rclcpp/rclcpp.hpp"
#include "moveit/move_group_interface/move_group_interface.h"
#include "geometry_msgs/msg/twist_stamped.hpp" // Still needed for twist_pub_ if not removed

// Forward declaration
namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(rclcpp::Node::SharedPtr node, QWidget *parent = nullptr);
    ~MainWindow();

private slots:
    void updateRobotState();
    void goHome();
    void planPose(); // New slot for planning
    void executePlan(); // New slot for executing plan

private:
    void setupUi();

    rclcpp::Node::SharedPtr node_;
    moveit::planning_interface::MoveGroupInterface move_group_interface_;
    moveit::planning_interface::MoveGroupInterface::Plan current_plan_; // To store the planned trajectory
    
    // UI Elements
    QLabel *joint_values_label_;
    QLabel *pose_values_label_;
    QPushButton *home_button_;

    // New UI Elements for Cartesian Pose Input
    QLineEdit *x_input_;
    QLineEdit *y_input_;
    QLineEdit *z_input_;
    QLineEdit *rx_input_;
    QLineEdit *ry_input_;
    QLineEdit *rz_input_;
    QPushButton *plan_button_;
    QPushButton *execute_button_;

    // ROS2
    QTimer *ros_timer_;
};

#endif // MAIN_WINDOW_H