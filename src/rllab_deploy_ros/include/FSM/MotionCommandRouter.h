// Copyright (c) 2025, Unitree Robotics Co., Ltd.
// All rights reserved.

#pragma once

#include <atomic>
#include <string>

#include <spdlog/spdlog.h>

#include "FSM/BaseState.h"

class MotionCommandRouter
{
public:
    static void RequestState(int state_id)
    {
        if (state_id == 0) {
            return;
        }
        requested_state().store(state_id, std::memory_order_relaxed);
    }

    static bool RequestStateByName(const std::string& state_name)
    {
        auto it = FSMStringMap.right.find(state_name);
        if (it == FSMStringMap.right.end()) {
            spdlog::warn("MotionCommandRouter: unknown state name '{}'", state_name);
            return false;
        }
        RequestState(it->second);
        return true;
    }

    static int ConsumeRequest()
    {
        return requested_state().exchange(0, std::memory_order_relaxed);
    }

private:
    static std::atomic<int>& requested_state()
    {
        static std::atomic<int> value{0};
        return value;
    }
};
