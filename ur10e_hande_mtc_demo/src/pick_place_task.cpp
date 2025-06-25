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
    // UR10e만 사용 (그리퍼 문제 해결할 때까지)
    arm_group_name_ = "ur_manipulator";
    // eef_name_ = "hande_ee";        // 임시로 주석 처리
    // hand_group_name_ = "hande_gripper";  // 임시로 주석 처리
    hand_frame_ = "tool0";  // robotiq_hande_end 대신 tool0
    
    // 객체 설정
    object_name_ = "object";
    object_reference_frame_ = "base_link";
    surface_link_ = "table";
    object_dimensions_ = {0.025, 0.025, 0.025};
    
    // 테이블 설정
    table_dimensions_ = {0.4, 0.6, 0.03};
}

bool PickPlaceTask::init(rclcpp::Node::SharedPtr node) {
    node_ = node;
    
    RCLCPP_INFO(LOGGER, "UR10e + Hande Pick/Place 태스크 초기화");
    
    try {
        // robot_description도 함께 복사해야 함
        auto temp_node = rclcpp::Node::make_shared("temp_param_node");
        auto param_client = std::make_shared<rclcpp::SyncParametersClient>(temp_node, "/move_group");
        
        if (param_client->wait_for_service(std::chrono::seconds(5))) {
            try {
                // robot_description과 robot_description_semantic 모두 복사
                auto robot_desc = param_client->get_parameter<std::string>("robot_description");
                auto robot_desc_semantic = param_client->get_parameter<std::string>("robot_description_semantic");
                
                // 현재 노드에 파라미터 설정
                if (!node->has_parameter("robot_description")) {
                    node->declare_parameter("robot_description", robot_desc);
                }
                if (!node->has_parameter("robot_description_semantic")) {
                    node->declare_parameter("robot_description_semantic", robot_desc_semantic);
                }
                
                RCLCPP_INFO(LOGGER, "MoveIt 파라미터 복사 완료");
            } catch (const std::exception& e) {
                RCLCPP_ERROR(LOGGER, "파라미터 복사 실패: %s", e.what());
                return false;
            }
        } else {
            RCLCPP_ERROR(LOGGER, "move_group 서비스에 연결할 수 없습니다");
            return false;
        }
        
        // 로봇 모델 로더 생성 시 올바른 파라미터 사용
        robot_model_loader::RobotModelLoader robot_model_loader(node, "robot_description");
        auto robot_model = robot_model_loader.getModel();
        
        if (!robot_model) {
            RCLCPP_ERROR(LOGGER, "로봇 모델 로드 실패");
            return false;
        }
        
        // 로봇 모델 이름 확인
        RCLCPP_INFO(LOGGER, "로드된 로봇 모델: %s", robot_model->getName().c_str());
        
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
        // OMPL 명시적 설정
        sampling_planner_ = std::make_shared<solvers::PipelinePlanner>(node_, "ompl");
        if (!sampling_planner_) {
            throw std::runtime_error("sampling_planner 생성 실패");
        }
        
        // OMPL 설정을 명시적으로 지정
        sampling_planner_->setProperty("planning_plugin", "ompl_interface/OMPLPlanner");
        sampling_planner_->setProperty("goal_joint_tolerance", 1e-4);
        sampling_planner_->setProperty("planning_attempts", 10);
        sampling_planner_->setProperty("planning_time", 5.0);
        
        // 직교 경로 플래너
        cartesian_planner_ = std::make_shared<solvers::CartesianPath>();
        if (!cartesian_planner_) {
            throw std::runtime_error("cartesian_planner 생성 실패");
        }
        cartesian_planner_->setMaxVelocityScalingFactor(1.0);
        cartesian_planner_->setMaxAccelerationScalingFactor(1.0);
        cartesian_planner_->setStepSize(.01);
        
        RCLCPP_INFO(LOGGER, "플래닝 파이프라인 설정 완료 (OMPL 강제 사용)");
        
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
    table_pose_.pose.position.x = 0.6;    // 전방 40cm
    table_pose_.pose.position.y = 0.0;    // 중앙
    table_pose_.pose.position.z = 0.2;    // 베이스에서 40cm 위 (총 1.4m 높이)
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
        // UR만 사용하도록 수정 (그리퍼 문제 해결할 때까지)
        task_.setProperty("group", "ur_manipulator");  // hande_ee 대신 ur_manipulator만
        // task_.setProperty("eef", eef_name_);  // 임시로 주석 처리
        task_.setProperty("ik_frame", "tool0");  // robotiq_hande_end 대신 tool0

        RCLCPP_INFO(LOGGER, "태스크 속성 설정 완료: group=ur_manipulator, ik_frame=tool0");

        // 현재 상태 설정
        {
            auto stage = std::make_unique<stages::CurrentState>("current");
            stage->setTimeout(15.0);
            task_.add(std::move(stage));
        }

        // 안전한 ready 포즈로 이동
        {
            auto stage = std::make_unique<stages::MoveTo>("move to ready", sampling_planner_);
            stage->setGroup("ur_manipulator");
            
            // 충돌이 적은 안전한 포즈 설정 (실제 로봇의 현재 위치 고려)
            std::map<std::string, double> ready_pose;
            ready_pose["shoulder_pan_joint"] = 0.0;
            ready_pose["shoulder_lift_joint"] = -1.0;
            ready_pose["elbow_joint"] = 1.0;
            ready_pose["wrist_1_joint"] = -1.0;
            ready_pose["wrist_2_joint"] = -1.57;
            ready_pose["wrist_3_joint"] = 0.0;
            
            stage->setGoal(ready_pose);
            stage->setTimeout(10.0);
            task_.add(std::move(stage));
        }

        RCLCPP_INFO(LOGGER, "UR 전용 태스크 로드 완료 (그리퍼 제거)");
        
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
    
    // 첫 번째 해결책을 RViz에 발행 (시각화용)
    if (!task_.solutions().empty()) {
        try {
            task_.introspection().publishSolution(*task_.solutions().front());
            RCLCPP_INFO(LOGGER, "해결책이 RViz에 발행되었습니다");
        } catch (const std::exception& e) {
            RCLCPP_WARN(LOGGER, "해결책 발행 실패: %s", e.what());
        }
    }
    
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