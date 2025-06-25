/*********************************************************************
 * UR10e + Hande 그리퍼 MTC Pick/Place 데모
 *********************************************************************/

#include <rclcpp/rclcpp.hpp>
#include <ur10e_hande_mtc_demo/pick_place_task.h>

static const rclcpp::Logger LOGGER = rclcpp::get_logger("ur10e_hande_mtc_demo");

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    
    rclcpp::NodeOptions node_options;
    node_options.automatically_declare_parameters_from_overrides(true);
    auto node = rclcpp::Node::make_shared("ur10e_hande_mtc_demo", node_options);
    
    // MTC 시각화를 위한 스피너 시작
    std::thread spinning_thread([node] { rclcpp::spin(node); });
    
    RCLCPP_INFO(LOGGER, "UR10e + Hande MTC Pick/Place 데모 시작 (계획만)");
    
    // move_group이 완전히 시작될 때까지 대기
    RCLCPP_INFO(LOGGER, "move_group 준비 대기 중...");
    rclcpp::sleep_for(std::chrono::seconds(5));
    
    // 데모 씬 설정
    ur10e_hande_mtc_demo::setupDemoScene(node);
    
    // Pick/Place 태스크 생성 및 계획
    ur10e_hande_mtc_demo::PickPlaceTask pick_place_task("ur10e_hande_pick_place_task");
    
    if (!pick_place_task.init(node)) {
        RCLCPP_ERROR(LOGGER, "태스크 초기화 실패");
        spinning_thread.join();
        return 1;
    }

    if (pick_place_task.plan(10)) {
        RCLCPP_INFO(LOGGER, "계획 성공! RViz에서 결과를 확인하세요.");
        RCLCPP_INFO(LOGGER, "프로그램을 유지합니다... (Ctrl+C로 종료)");
    } else {
        RCLCPP_ERROR(LOGGER, "태스크 계획 실패");
    }

    // RViz 시각화를 위해 계속 실행
    spinning_thread.join();
    return 0;
}