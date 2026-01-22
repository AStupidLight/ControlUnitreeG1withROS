#include "FSM/CtrlFSM.h"
#include "FSM/State_Passive.h"
#include "FSM/State_FixStand.h"
#include "FSM/State_RLBase.h"
#include "State_Mimic.h"
#include "motion_node.h"

#include <rclcpp/rclcpp.hpp>
#include <thread>

std::unique_ptr<LowCmd_t> FSMState::lowcmd = nullptr;
std::shared_ptr<LowState_t> FSMState::lowstate = nullptr;
std::shared_ptr<Keyboard> FSMState::keyboard = std::make_shared<Keyboard>();

void init_fsm_state()
{
    auto lowcmd_sub = std::make_shared<unitree::robot::g1::subscription::LowCmd>();
    usleep(0.2 * 1e6);
    if(!lowcmd_sub->isTimeout())
    {
        spdlog::critical("The other process is using the lowcmd channel, please close it first.");
        unitree::robot::go2::shutdown();
        exit(1);  // 恢复 exit，避免程序继续执行导致未定义行为
    }
    FSMState::lowcmd = std::make_unique<LowCmd_t>();
    FSMState::lowstate = std::make_shared<LowState_t>();
    spdlog::info("Waiting for connection to robot...");
    FSMState::lowstate->wait_for_connection();
    spdlog::info("Connected to robot.");
}

int main(int argc, char** argv)
{
    // Load parameters
    auto vm = param::helper(argc, argv);

    std::cout << " --- Unitree Robotics --- \n";
    std::cout << "     G1-29dof Controller \n";

    // Unitree DDS Config
    unitree::robot::ChannelFactory::Instance()->Init(0, vm["network"].as<std::string>());

    init_fsm_state();
    spdlog::info("[DEBUG] init_fsm_state() completed");

    FSMState::lowcmd->msg_.mode_machine() = 5; // 29dof
    spdlog::info("[DEBUG] Checking robot type...");
    if(!FSMState::lowcmd->check_mode_machine(FSMState::lowstate)) {
        spdlog::critical("Unmatched robot type.");
        exit(-1);
    }
    spdlog::info("[DEBUG] Robot type check passed");
    
    // Initialize FSM
    spdlog::info("[DEBUG] Initializing FSM...");
    auto fsm = std::make_unique<CtrlFSM>(param::config["FSM"]);
    fsm->start();
    spdlog::info("[DEBUG] FSM started");

    spdlog::info("[DEBUG] Initializing ROS2 node...");
    rclcpp::init(argc, argv);
    auto motion_node = std::make_shared<MotionNode>();
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(motion_node);
    std::thread ros_thread([&executor]() { executor.spin(); });
    spdlog::info("[DEBUG] ROS2 node running");

    std::cout << "Press [L2 + Up] to enter FixStand mode.\n";
    std::cout << "And then press [R1 + X] to start controlling the robot.\n";

    while (true)
    {
        sleep(1);
    }
    
    return 0;
}
