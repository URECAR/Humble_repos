#include "ur10e_hande_gui/main_window.h"
#include <QApplication>
#include <thread>
#include <iomanip>
#include <sstream>
#include <QLineEdit>
#include <QDoubleValidator>
#include <QMessageBox>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <QVBoxLayout> // Added
#include <QGridLayout> // Added
#include <QHBoxLayout> // Added

MainWindow::MainWindow(rclcpp::Node::SharedPtr node, QWidget *parent)
    : QMainWindow(parent), node_(node), move_group_interface_(node, "ur_manipulator")
{
    setupUi();

    // Timer to update robot state
    ros_timer_ = new QTimer(this);
    connect(ros_timer_, &QTimer::timeout, this, &MainWindow::updateRobotState);
    ros_timer_->start(100); // Update every 100ms
}

MainWindow::~MainWindow()
{
    // Destructor implementation
}

void MainWindow::setupUi()
{
    // Main widget and layout
    QWidget *central_widget = new QWidget(this);
    QVBoxLayout *main_layout = new QVBoxLayout(central_widget);
    
    QGridLayout *state_layout = new QGridLayout();
    state_layout->addWidget(new QLabel("<b>Joint Values:</b>"), 0, 0);
    joint_values_label_ = new QLabel("N/A");
    state_layout->addWidget(joint_values_label_, 0, 1);
    
    state_layout->addWidget(new QLabel("<b>End-Effector Pose:</b>"), 1, 0);
    pose_values_label_ = new QLabel("N/A");
    state_layout->addWidget(pose_values_label_, 1, 1);

    main_layout->addLayout(state_layout);

    home_button_ = new QPushButton("Go Home");
    main_layout->addWidget(home_button_);
    connect(home_button_, &QPushButton::clicked, this, &MainWindow::goHome);

    // New: Cartesian Pose Input
    main_layout->addWidget(new QLabel("<b>Target Cartesian Pose (m, rad):</b>"));
    QGridLayout *pose_input_layout = new QGridLayout();

    pose_input_layout->addWidget(new QLabel("X:"), 0, 0);
    x_input_ = new QLineEdit();
    x_input_->setValidator(new QDoubleValidator(-1000.0, 1000.0, 3, this));
    pose_input_layout->addWidget(x_input_, 0, 1);

    pose_input_layout->addWidget(new QLabel("Y:"), 1, 0);
    y_input_ = new QLineEdit();
    y_input_->setValidator(new QDoubleValidator(-1000.0, 1000.0, 3, this));
    pose_input_layout->addWidget(y_input_, 1, 1);

    pose_input_layout->addWidget(new QLabel("Z:"), 2, 0);
    z_input_ = new QLineEdit();
    z_input_->setValidator(new QDoubleValidator(-1000.0, 1000.0, 3, this));
    pose_input_layout->addWidget(z_input_, 2, 1);

    pose_input_layout->addWidget(new QLabel("Roll (RX):"), 0, 2);
    rx_input_ = new QLineEdit();
    rx_input_->setValidator(new QDoubleValidator(-M_PI, M_PI, 3, this));
    pose_input_layout->addWidget(rx_input_, 0, 3);

    pose_input_layout->addWidget(new QLabel("Pitch (RY):"), 1, 2);
    ry_input_ = new QLineEdit();
    ry_input_->setValidator(new QDoubleValidator(-M_PI, M_PI, 3, this));
    pose_input_layout->addWidget(ry_input_, 1, 3);

    pose_input_layout->addWidget(new QLabel("Yaw (RZ):"), 2, 2);
    rz_input_ = new QLineEdit();
    rz_input_->setValidator(new QDoubleValidator(-M_PI, M_PI, 3, this));
    pose_input_layout->addWidget(rz_input_, 2, 3);

    main_layout->addLayout(pose_input_layout);

    plan_button_ = new QPushButton("Plan");
    execute_button_ = new QPushButton("Execute");
    execute_button_->setEnabled(false); // Disable execute until plan is successful

    QHBoxLayout *action_buttons_layout = new QHBoxLayout();
    action_buttons_layout->addWidget(plan_button_);
    action_buttons_layout->addWidget(execute_button_);
    main_layout->addLayout(action_buttons_layout);

    connect(plan_button_, &QPushButton::clicked, this, &MainWindow::planPose);
    connect(execute_button_, &QPushButton::clicked, this, &MainWindow::executePlan);

    setCentralWidget(central_widget);
    setWindowTitle("UR10e Hand-E Control");
}

void MainWindow::updateRobotState()
{
    std::vector<double> joint_values = move_group_interface_.getCurrentJointValues();
    std::stringstream joint_ss;
    joint_ss << std::fixed << std::setprecision(2);
    for(const auto& val : joint_values) { joint_ss << val << " "; }
    joint_values_label_->setText(QString::fromStdString(joint_ss.str()));

    geometry_msgs::msg::PoseStamped current_pose = move_group_interface_.getCurrentPose();
    std::stringstream pose_ss;
    pose_ss << std::fixed << std::setprecision(3); // Increased precision for pose
    pose_ss << "P: [X:" << current_pose.pose.position.x << ", Y:" << current_pose.pose.position.y << ", Z:" << current_pose.pose.position.z << "] | ";
    pose_ss << "O: [X:" << current_pose.pose.orientation.x << ", Y:" << current_pose.pose.orientation.y << ", Z:" << current_pose.pose.orientation.z << ", W:" << current_pose.pose.orientation.w << "]";
    pose_values_label_->setText(QString::fromStdString(pose_ss.str()));
}

void MainWindow::goHome()
{
    // Run moveit commands in a separate thread to not block the GUI
    std::thread([this]() {
        move_group_interface_.setNamedTarget("home");
        move_group_interface_.move();
    }).detach();
}

void MainWindow::planPose()
{
    execute_button_->setEnabled(false); // Disable execute button until new plan is successful

    std::thread([this]() {
        geometry_msgs::msg::Pose target_pose;
        try {
            target_pose.position.x = x_input_->text().toDouble();
            target_pose.position.y = y_input_->text().toDouble();
            target_pose.position.z = z_input_->text().toDouble();
            
            // Convert RPY to Quaternion
            tf2::Quaternion q;
            q.setRPY(rx_input_->text().toDouble(), ry_input_->text().toDouble(), rz_input_->text().toDouble());
            target_pose.orientation.x = q.x();
            target_pose.orientation.y = q.y();
            target_pose.orientation.z = q.z();
            target_pose.orientation.w = q.w();
        } catch (const std::exception& e) {
            QMetaObject::invokeMethod(this, [e_msg = e.what()]() {
                QMessageBox::critical(nullptr, "Input Error", QString("Invalid input: %1").arg(e_msg));
            }, Qt::QueuedConnection);
            return;
        }

        move_group_interface_.setPoseTarget(target_pose);

        moveit::planning_interface::MoveGroupInterface::Plan my_plan;
        bool success = (move_group_interface_.plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);

        QMetaObject::invokeMethod(this, [this, success, my_plan_copy = my_plan]() mutable {
            if (success) {
                QMessageBox::information(this, "Plan Successful", "Motion plan generated successfully!");
                current_plan_ = my_plan_copy;
                execute_button_->setEnabled(true);
            } else {
                QMessageBox::warning(this, "Plan Failed", "Failed to generate motion plan.");
                execute_button_->setEnabled(false);
            }
        }, Qt::QueuedConnection);
    }).detach();
}

void MainWindow::executePlan()
{
    execute_button_->setEnabled(false); // Disable execute button during execution
    plan_button_->setEnabled(false); // Disable plan button during execution

    std::thread([this]() {
        moveit::core::MoveItErrorCode result = move_group_interface_.execute(current_plan_);

        QMetaObject::invokeMethod(this, [this, result]() {
            if (result == moveit::core::MoveItErrorCode::SUCCESS) {
                QMessageBox::information(this, "Execution Successful", "Motion plan executed successfully!");
            } else {
                QMessageBox::critical(this, "Execution Failed", "Failed to execute motion plan.");
            }
            plan_button_->setEnabled(true);
        }, Qt::QueuedConnection);
    }).detach();
}

// Main function
int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("ur10e_hande_gui_node", rclcpp::NodeOptions().use_intra_process_comms(true));

    QApplication app(argc, argv);
    
    MainWindow window(node);
    window.show();

    std::thread ros_thread([&]() {
        rclcpp::spin(node);
    });

    int result = app.exec();

    rclcpp::shutdown();
    ros_thread.join();
    
    return result;
}
