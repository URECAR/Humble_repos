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
    
    std::thread spinning_thread([node] { rclcpp::spin(node); });

    // 파라미터 선언 및 기본값 설정 (중복 선언 방지)
    bool execute = true;
    int max_solutions = 10;
    
    try {
        if (!node->has_parameter("execute")) {
            node->declare_parameter("execute", execute);
        }
        if (!node->has_parameter("max_solutions")) {
            node->declare_parameter("max_solutions", max_solutions);
        }
        
        // 파라미터 가져오기
        execute = node->get_parameter("execute").as_bool();
        max_solutions = node->get_parameter("max_solutions").as_int();
        
    } catch (const std::exception& e) {
        RCLCPP_WARN(LOGGER, "파라미터 설정 중 오류: %s. 기본값 사용.", e.what());
    }
    
    RCLCPP_INFO(LOGGER, "UR10e + Hande MTC Pick/Place 데모 시작");
    RCLCPP_INFO(LOGGER, "실행 모드: %s", execute ? "실행" : "계획만");
    
    // 데모 씬 설정
    ur10e_hande_mtc_demo::setupDemoScene(node);
    
    // Pick/Place 태스크 생성 및 실행
    ur10e_hande_mtc_demo::PickPlaceTask pick_place_task("ur10e_hande_pick_place");
    
    if (!pick_place_task.init(node)) {
        RCLCPP_ERROR(LOGGER, "태스크 초기화 실패");
        return 1;
    }

    if (pick_place_task.plan(max_solutions)) {
        RCLCPP_INFO(LOGGER, "계획 성공!");
        if (execute) {
            RCLCPP_INFO(LOGGER, "태스크 실행 중...");
            if (pick_place_task.execute()) {
                RCLCPP_INFO(LOGGER, "태스크 실행 완료!");
            } else {
                RCLCPP_ERROR(LOGGER, "태스크 실행 실패");
            }
        } else {
            RCLCPP_INFO(LOGGER, "실행 모드가 비활성화됨");
        }
    } else {
        RCLCPP_ERROR(LOGGER, "태스크 계획 실패");
    }

    // 인트로스펙션 유지
    spinning_thread.join();
    return 0;
}