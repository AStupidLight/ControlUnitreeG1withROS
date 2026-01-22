#include "FSM/State_RLBase.h"
#include "unitree_articulation.h"
#include "isaaclab/envs/mdp/observations/observations.h"
#include "isaaclab/envs/mdp/actions/joint_actions.h"
#include <unordered_map>

namespace isaaclab
{
// keyboard velocity commands example
// change "velocity_commands" observation name in policy deploy.yaml to "keyboard_velocity_commands"
REGISTER_OBSERVATION(keyboard_velocity_commands)
{
    std::string key = FSMState::keyboard->key();
    static auto cfg = env->cfg["commands"]["base_velocity"]["ranges"];

    static std::unordered_map<std::string, std::vector<float>> key_commands = {
        {"w", {1.0f, 0.0f, 0.0f}},
        {"s", {-1.0f, 0.0f, 0.0f}},
        {"a", {0.0f, 1.0f, 0.0f}},
        {"d", {0.0f, -1.0f, 0.0f}},
        {"q", {0.0f, 0.0f, 1.0f}},
        {"e", {0.0f, 0.0f, -1.0f}}
    };
    std::vector<float> cmd = {0.0f, 0.0f, 0.0f};
    if (key_commands.find(key) != key_commands.end())
    {
        // TODO: smooth and limit the velocity commands
        cmd = key_commands[key];
    }
    return cmd;
}

}

State_RLBase::State_RLBase(int state_mode, std::string state_string)
: FSMState(state_mode, state_string) 
{
    auto cfg = param::config["FSM"][state_string];
    auto policy_dir_raw = cfg["policy_dir"].as<std::string>();
    auto policy_dir = param::parser_policy_dir(policy_dir_raw);
    
    // 提取policy版本信息
    std::string policy_version = "unknown";
    if (policy_dir_raw.find("/v1") != std::string::npos || policy_dir_raw.find("\\v1") != std::string::npos) {
        policy_version = "v1";
    } else if (policy_dir_raw.find("/v0") != std::string::npos || policy_dir_raw.find("\\v0") != std::string::npos) {
        policy_version = "v0";
    } else {
        // 尝试从最终路径中提取版本
        std::string policy_dir_str = policy_dir.string();
        size_t v1_pos = policy_dir_str.find("/v1/");
        size_t v0_pos = policy_dir_str.find("/v0/");
        if (v1_pos != std::string::npos) {
            policy_version = "v1";
        } else if (v0_pos != std::string::npos) {
            policy_version = "v0";
        }
    }
    
    spdlog::info("========================================");
    spdlog::info("State: {} -> Using Policy Version: {}", state_string, policy_version);
    spdlog::info("Policy directory: {}", policy_dir.string());
    spdlog::info("========================================");
    
    env = std::make_unique<isaaclab::ManagerBasedRLEnv>(
        YAML::LoadFile(policy_dir / "params" / "deploy.yaml"),
        std::make_shared<unitree::BaseArticulation<LowState_t::SharedPtr>>(FSMState::lowstate)
    );
    env->alg = std::make_unique<isaaclab::OrtRunner>(policy_dir / "exported" / "policy.onnx");

    this->registered_checks.emplace_back(
        std::make_pair(
            [&]()->bool{ return isaaclab::mdp::bad_orientation(env.get(), 1.0); },
            FSMStringMap.right.at("Passive")
        )
    );
}

void State_RLBase::run()
{
    auto action = env->action_manager->processed_actions();
    for(int i(0); i < env->robot->data.joint_ids_map.size(); i++) {
        lowcmd->msg_.motor_cmd()[env->robot->data.joint_ids_map[i]].q() = action[i];
    }
}
