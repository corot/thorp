#pragma once

#include <string>

#include <thorp_msgs/msg/thorp_error.hpp>

namespace thorp::bt
{
/**
 * What an `error` output port carries: a code for trees and callers to act on, and a one-line reason for whoever
 * reads it.
 */
inline thorp_msgs::msg::ThorpError makeError(int32_t code, const std::string& text)
{
  thorp_msgs::msg::ThorpError error;
  error.code = code;
  error.text = text;
  return error;
}

}  // namespace thorp::bt
