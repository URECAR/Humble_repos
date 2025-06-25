#include <ur10e_hande_mtc_demo/pick_place_task.h>

#include <moveit/task_constructor/stages/current_state.h>
#include <moveit/task_constructor/stages/move_to.h>
#include <moveit/task_constructor/stages/move_relative.h>
#include <moveit/task_constructor/stages/connect.h>
#include <moveit/task_constructor/stages/compute_ik.h>
#include <moveit/task_constructor/stages/modify_planning_scene.h>
#include <moveit/task_constructor/stages/generate_grasp_pose.h>
#include <moveit/task_constructor/stages/generate_pose.h>
#include <moveit/task_constructor/stages/generate_place_pose.h>

#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_visual_tools/moveit_visual_tools.h>

#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace ur10e_hande_mtc_demo {

using namespace moveit::task_constructor;

static const rclcpp::Logger LOGGER = rclcpp::get_logger("pick_place_task");

PickPlaceTask::PickPlaceTask(const std::string& task_name) : task_(task_name) {
    // UR10e + Hande 구성
    arm_group_name_ = "ur_manipulator";
    eef_name_ = "hande_ee";
    hand_group_name_ = "hande_gripper";
    hand_frame_ = "robotiq_hande_end";
    
    // 객체 설정
    object_name_ = "object";
    object_reference_frame_ = "base_link";
    surface_link_ = "table";
    object_dimensions_ = {0.025, 0.025, 0.025};  // 2.5cm 정육면체
    
    // 테이블 설정 (UR10e 베이스가 1m 높이)
    table_dimensions_ = {0.4, 0.6, 0.03};  // 40x60x3cm 테이블
}

bool PickPlaceTask::init(rclcpp::Node::SharedPtr node) {
    node_ = node;
    
    RCLCPP_INFO(LOGGER, "UR10e + Hande Pick/Place 태스크 초기화");
    
    try {
        // 먼저 파라미터 복사 (다른 노드에서 글로벌로)
        auto param_client = std::make_shared<rclcpp::SyncParametersClient>(node, "/move_group");
        
        // move_group에서 파라미터 가져오기
        if (param_client->wait_for_service(std::chrono::seconds(5))) {
            try {
                auto robot_desc = param_client->get_parameter<std::string>("robot_description");
                auto robot_desc_semantic = param_client->get_parameter<std::string>("robot_description_semantic");
                
                // 현재 노드에 파라미터 설정
                node->declare_parameter("robot_description", robot_desc);
                node->declare_parameter("robot_description_semantic", robot_desc_semantic);
                
                RCLCPP_INFO(LOGGER, "MoveIt 파라미터를 성공적으로 복사했습니다");
            } catch (const std::exception& e) {
                RCLCPP_WARN(LOGGER, "파라미터 복사 중 오류: %s", e.what());
            }
        } else {
            RCLCPP_WARN(LOGGER, "move_group 파라미터 서비스에 연결할 수 없습니다");
        }
        
        // 로봇 모델 로더 생성
        robot_model_loader::RobotModelLoader robot_model_loader(node, "robot_description");
        auto robot_model = robot_model_loader.getModel();
        
        if (!robot_model) {
            RCLCPP_ERROR(LOGGER, "로봇 모델 로드 실패 - MoveIt이 실행되지 않았을 수 있습니다");
            RCLCPP_ERROR(LOGGER, "해결 방법: 먼저 다음을 실행하세요:");
            RCLCPP_ERROR(LOGGER, "ros2 launch ur10e_hande_moveit_config move_group.launch.py");
            throw std::runtime_error("로봇 모델 로드 실패");
        }
        
        // 태스크에 로봇 모델 설정
        task_.setRobotModel(robot_model);
        
        RCLCPP_INFO(LOGGER, "로봇 모델 로드 완료");
        
        // 계획 파이프라인 설정
        setupPlanningPipeline();
        
        // 태스크 로드
        loadTask();
        
        RCLCPP_INFO(LOGGER, "태스크 초기화 완료");
        return true;
        
    } catch (const std::exception& e) {
        RCLCPP_ERROR(LOGGER, "태스크 초기화 중 오류 발생: %s", e.what());
        return false;
    }
}

void PickPlaceTask::setupPlanningPipeline() {
    try {
        // OMPL 샘플링 플래너
        sampling_planner_ = std::make_shared<solvers::PipelinePlanner>(node_);
        if (!sampling_planner_) {
            throw std::runtime_error("sampling_planner 생성 실패");
        }
        sampling_planner_->setProperty("goal_joint_tolerance", 1e-5);
        
        // 직교 경로 플래너
        cartesian_planner_ = std::make_shared<solvers::CartesianPath>();
        if (!cartesian_planner_) {
            throw std::runtime_error("cartesian_planner 생성 실패");
        }
        cartesian_planner_->setMaxVelocityScalingFactor(1.0);
        cartesian_planner_->setMaxAccelerationScalingFactor(1.0);
        cartesian_planner_->setStepSize(.01);
        
        RCLCPP_INFO(LOGGER, "플래닝 파이프라인 설정 완료");
        
    } catch (const std::exception& e) {
        RCLCPP_ERROR(LOGGER, "플래닝 파이프라인 설정 오류: %s", e.what());
        throw;
    }
}

void PickPlaceTask::setupDemoScene() {
    moveit::planning_interface::PlanningSceneInterface psi;
    
    // 테이블 추가
    moveit_msgs::msg::CollisionObject table;
    table.id = surface_link_;
    table.header.frame_id = object_reference_frame_;
    table.primitives.resize(1);
    table.primitives[0].type = shape_msgs::msg::SolidPrimitive::BOX;
    table.primitives[0].dimensions = {table_dimensions_[0], table_dimensions_[1], table_dimensions_[2]};
    
    table_pose_.header.frame_id = object_reference_frame_;
    table_pose_.pose.position.x = 0.4;    // 전방 40cm
    table_pose_.pose.position.y = 0.0;    // 중앙
    table_pose_.pose.position.z = 0.4;    // 베이스에서 40cm 위 (총 1.4m 높이)
    table_pose_.pose.orientation.w = 1.0;
    
    table.primitive_poses.push_back(table_pose_.pose);
    
    // 객체 추가
    moveit_msgs::msg::CollisionObject object;
    object.id = object_name_;
    object.header.frame_id = object_reference_frame_;
    object.primitives.resize(1);
    object.primitives[0].type = shape_msgs::msg::SolidPrimitive::BOX;
    object.primitives[0].dimensions = {object_dimensions_[0], object_dimensions_[1], object_dimensions_[2]};
    
    object_pose_.header.frame_id = object_reference_frame_;
    object_pose_.pose.position.x = table_pose_.pose.position.x;
    object_pose_.pose.position.y = table_pose_.pose.position.y + 0.1;  // 테이블 위 10cm 오른쪽
    object_pose_.pose.position.z = table_pose_.pose.position.z + 
                                  0.5 * table_dimensions_[2] + 
                                  0.5 * object_dimensions_[2];  // 테이블 위
    object_pose_.pose.orientation.w = 1.0;
    
    object.primitive_poses.push_back(object_pose_.pose);
    
    // 배치 위치 설정
    place_pose_.header.frame_id = object_reference_frame_;
    place_pose_.pose.position.x = table_pose_.pose.position.x;
    place_pose_.pose.position.y = table_pose_.pose.position.y - 0.15;  // 테이블 위 15cm 왼쪽
    place_pose_.pose.position.z = object_pose_.pose.position.z;
    place_pose_.pose.orientation.w = 1.0;
    
    // 씬에 추가
    psi.applyCollisionObjects({table, object});
    
    RCLCPP_INFO(LOGGER, "데모 씬 설정 완료");
}

void PickPlaceTask::loadTask() {
    try {
        // 태스크 속성 먼저 설정
        task_.setProperty("group", arm_group_name_);
        task_.setProperty("eef", eef_name_);
        task_.setProperty("ik_frame", hand_frame_);

        RCLCPP_INFO(LOGGER, "태스크 속성 설정 완료: group=%s, eef=%s, ik_frame=%s", 
                   arm_group_name_.c_str(), eef_name_.c_str(), hand_frame_.c_str());

        // 현재 상태 설정 (Joint State가 안정화될 때까지 대기)
        {
            auto stage = std::make_unique<stages::CurrentState>("current");
            // 더 긴 대기 시간 설정
            stage->setTimeout(10.0);
            task_.add(std::move(stage));
        }

        // 간단한 이동 태스크만 추가 (더 안전한 테스트)
        {
            auto stage = std::make_unique<stages::MoveTo>("move to ready", sampling_planner_);
            stage->setGroup(arm_group_name_);
            
            // Named target 대신 특정 조인트 각도 사용
            std::map<std::string, double> ready_pose;
            ready_pose["shoulder_pan_joint"] = 0.0;
            ready_pose["shoulder_lift_joint"] = -1.57;
            ready_pose["elbow_joint"] = 1.57;
            ready_pose["wrist_1_joint"] = -1.57;
            ready_pose["wrist_2_joint"] = -1.57;
            ready_pose["wrist_3_joint"] = 0.0;
            
            stage->setGoal(ready_pose);
            task_.add(std::move(stage));
        }

        RCLCPP_INFO(LOGGER, "Pick/Place 태스크 로드 완료 (단순화 버전)");
        
    } catch (const std::exception& e) {
        RCLCPP_ERROR(LOGGER, "태스크 로드 중 오류 발생: %s", e.what());
        throw;
    }
}

bool PickPlaceTask::plan(size_t max_solutions) {
    RCLCPP_INFO(LOGGER, "Pick/Place 태스크 계획 시작...");
    
    // Joint State가 준비될 때까지 대기
    RCLCPP_INFO(LOGGER, "Joint State 연결 대기 중...");
    rclcpp::sleep_for(std::chrono::seconds(3));
    
    setupDemoScene();
    
    try {
        task_.init();
        RCLCPP_INFO(LOGGER, "태스크 초기화 완료");
    } catch (const std::exception& e) {
        RCLCPP_ERROR(LOGGER, "태스크 초기화 실패: %s", e.what());
        return false;
    }

    if (!task_.plan(max_solutions)) {
        RCLCPP_ERROR(LOGGER, "태스크 계획 실패");
        return false;
    }

    RCLCPP_INFO(LOGGER, "태스크 계획 성공! 해결책 개수: %zu", task_.solutions().size());
    return true;
}

bool PickPlaceTask::execute() {
    RCLCPP_INFO(LOGGER, "태스크 실행 시작...");
    
    auto result = task_.execute(*task_.solutions().front());
    if (result.val != moveit_msgs::msg::MoveItErrorCodes::SUCCESS) {
        RCLCPP_ERROR(LOGGER, "태스크 실행 실패");
        return false;
    }

    RCLCPP_INFO(LOGGER, "태스크 실행 성공!");
    return true;
}

void setupDemoScene(rclcpp::Node::SharedPtr node) {
    (void)node;  // unused parameter 경고 제거
    
    rclcpp::sleep_for(std::chrono::seconds(1));  // move_group 시작 대기
    
    moveit::planning_interface::PlanningSceneInterface psi;
    
    // 기존 객체 제거
    std::vector<std::string> object_ids = {"object", "table"};
    psi.removeCollisionObjects(object_ids);
    
    rclcpp::sleep_for(std::chrono::milliseconds(500));
    
    // 테이블 추가
    moveit_msgs::msg::CollisionObject table;
    table.id = "table";
    table.header.frame_id = "base_link";
    table.primitives.resize(1);
    table.primitives[0].type = shape_msgs::msg::SolidPrimitive::BOX;
    table.primitives[0].dimensions = {0.4, 0.6, 0.03};  // 40x60x3cm
    
    geometry_msgs::msg::Pose table_pose;
    table_pose.position.x = 0.4;   // 전방 40cm
    table_pose.position.y = 0.0;   // 중앙
    table_pose.position.z = 0.4;   // 베이스에서 40cm 위
    table_pose.orientation.w = 1.0;
    
    table.primitive_poses.push_back(table_pose);
    
    // 객체 추가
    moveit_msgs::msg::CollisionObject object;
    object.id = "object";
    object.header.frame_id = "base_link";
    object.primitives.resize(1);
    object.primitives[0].type = shape_msgs::msg::SolidPrimitive::BOX;
    object.primitives[0].dimensions = {0.025, 0.025, 0.025};  // 2.5cm 정육면체
    
    geometry_msgs::msg::Pose object_pose;
    object_pose.position.x = table_pose.position.x;
    object_pose.position.y = table_pose.position.y + 0.1;  // 테이블 위 10cm 오른쪽
    object_pose.position.z = table_pose.position.z + 0.015 + 0.0125;  // 테이블 위
    object_pose.orientation.w = 1.0;
    
    object.primitive_poses.push_back(object_pose);
    
    // 씬에 추가
    psi.applyCollisionObjects({table, object});
    
    RCLCPP_INFO(rclcpp::get_logger("setup_demo_scene"), "데모 씬 설정 완료");
}

}  // namespace ur10e_hande_mtc_demo