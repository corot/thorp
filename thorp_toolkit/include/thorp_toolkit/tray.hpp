#pragma once

#include <cmath>
#include <optional>
#include <string>
#include <vector>

#include <geometry_msgs/msg/pose_stamped.hpp>

#include "thorp_toolkit/geometry.hpp"
#include "thorp_toolkit/parameters.hpp"
#include "thorp_toolkit/tf2.hpp"

namespace thorp::toolkit
{
/**
 * Thorp's tray, as a grid of slots for placing objects. A slot is free if no planning scene object is on it, so the
 * tray needs no state of its own.
 */
class Tray
{
public:
  /**
   * Read the tray geometry from the toolkit node's parameters: tray.link, tray.side_x, tray.side_y, tray.slot, and
   * placing_height_on_tray, the height over the tray's surface (the tray frame) where objects are released.
   */
  Tray()
  {
    double side_x, side_y;
    getParam("tray.link", link_, std::string("tray_link"));
    getParam("tray.side_x", side_x, 0.14);
    getParam("tray.side_y", side_y, 0.14);
    getParam("tray.slot", slot_, 0.035);
    getParam("placing_height_on_tray", placing_height_, 0.008);
    slots_x_ = static_cast<int>(std::round(side_x / slot_ + 0.1));
    slots_y_ = static_cast<int>(std::round(side_y / slot_ + 0.1));
  }

  size_t capacity() const
  {
    return slots_x_ * slots_y_;
  }

  /**
   * Whether an object is on the tray.
   * @param object Planning scene object, or anything with a header and a pose
   */
  template <typename Object>
  bool onTray(const Object& object) const
  {
    return slotIndex(object).has_value();
  }

  /**
   * Placing poses of the free slots, on the tray frame, in filling order.
   * @param objects Planning scene objects
   */
  template <typename Objects>
  std::vector<geometry_msgs::msg::PoseStamped> freeSlots(const Objects& objects) const
  {
    std::vector<bool> occupied(capacity(), false);
    for (const auto& [id, object] : objects)
    {
      if (auto index = slotIndex(object))
        occupied[*index] = true;
    }

    std::vector<geometry_msgs::msg::PoseStamped> free_slots;
    for (int j = 0; j < slots_y_; ++j)
      for (int i = 0; i < slots_x_; ++i)
        if (!occupied[j * slots_x_ + i])
          free_slots.push_back(createPose((i - slots_x_ / 2.0 + 0.5) * slot_, (j - slots_y_ / 2.0 + 0.5) * slot_,
                                          placing_height_, 0.0, 0.0, 0.0, link_));
    return free_slots;
  }

private:
  /**
   * Index of the slot an object is on, if it's on the tray: within its sides, from slightly below its base to well
   * above it.
   */
  template <typename Object>
  std::optional<int> slotIndex(const Object& object) const
  {
    geometry_msgs::msg::PoseStamped pose;
    pose.header = object.header;
    pose.pose = object.pose;
    if (!TF2::instance().transformPose(link_, pose, pose))
      return std::nullopt;
    if (pose.pose.position.z < -0.01 || pose.pose.position.z > 0.1)
      return std::nullopt;
    const int i = static_cast<int>(std::floor(pose.pose.position.x / slot_ + slots_x_ / 2.0));
    const int j = static_cast<int>(std::floor(pose.pose.position.y / slot_ + slots_y_ / 2.0));
    if (i < 0 || i >= slots_x_ || j < 0 || j >= slots_y_)
      return std::nullopt;
    return j * slots_x_ + i;
  }

  std::string link_;
  double slot_;
  double placing_height_;
  int slots_x_;
  int slots_y_;
};

}  // namespace thorp::toolkit
