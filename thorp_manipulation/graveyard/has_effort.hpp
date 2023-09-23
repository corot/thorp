bool haveEffort() { return std::any_of(gripper_effort.begin(), gripper_effort.end(),
                     [](double x) { return x != 0; } ); }
