#pragma once

#include <ros/ros.h>

namespace thorp::toolkit
{

void waitForObjectsSpawning(ros::Duration timeout = ros::Duration::ZERO);

};  // namespace thorp::toolkit