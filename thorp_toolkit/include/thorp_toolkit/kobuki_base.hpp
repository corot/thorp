/*
 * Author: Jorge Santos
 */

#pragma once

#include <kobuki_msgs/BumperEvent.h>

namespace thorp::toolkit
{

/**
 * @brief Return the name of the given bumper (left, center or right)
 * @param bumper Bumper index
 * @return Bumper name
 */
std::string bumperName(uint8_t bumper);

/**
 * @brief Return the name of the given bumper action (pressed or released)
 * @param bumper Bumper action index
 * @return Bumper action name
 */
std::string bumperAction(uint8_t action);

}  // namespace thorp::toolkit