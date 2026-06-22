#pragma once

// Public C++ facade for the libjaka bridge.
//
// Include the cxx-generated header from `src/ffi/to_cxx.rs` in real build
// outputs. This file keeps the driver-facing API aligned with
// robot_behavior.hpp and documents the stable surface expected by clients.

#include "robot_behavior.hpp"

#include <array>
#include <memory>
#include <string>

namespace libjaka {

using PoseData = robot_behavior::PoseData;
using PoseKind = robot_behavior::PoseKind;
using LoadState = robot_behavior::LoadState;

class JakaMini2 : public robot_behavior::Arm<6>,
                  public robot_behavior::JointMotion<6>,
                  public robot_behavior::FlangeMotion {
public:
  std::string version() const override;
  void init() override;
  void shutdown() override;
  void enable() override;
  void disable() override;
  void reset() override;
  void stop() override;
  void emergency_stop() override;
  void clear_emergency_stop() override;
  bool is_moving() override;

  std::string state() override;
  void set_load(const LoadState &load) override;
  std::array<double, 6> get_joint() const override;
  PoseData get_endpoint() const override;

  void move_joint(std::array<double, 6> target) override;
  void move_joint_sync(std::array<double, 6> target) override;
  void move_flange(PoseData target) override;
  void move_flange_sync(PoseData target) override;
};

std::unique_ptr<JakaMini2> jaka_mini2_attach(const std::string &ip);

} // namespace libjaka
