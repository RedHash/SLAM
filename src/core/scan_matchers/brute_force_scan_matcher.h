#ifndef SLAM_CTOR_CORE_BRUTE_FORCE_SCAN_MATCHER_H
#define SLAM_CTOR_CORE_BRUTE_FORCE_SCAN_MATCHER_H

#include <cassert>
#include <iostream>

#include "pose_enumeration_scan_matcher.h"

// TODO: move to pose enumerators
// FIXME: class name
class BruteForcePoseEnumerator : public PoseEnumerator {
public:
  BruteForcePoseEnumerator(double from_x, double to_x, double step_x,
                           double from_y, double to_y, double step_y,
                           double from_t, double to_t, double step_t)
    : _base_pose_is_set{false}
    , _from_x{from_x}, _to_x{to_x}, _step_x{step_x}
    , _from_y{from_y}, _to_y{to_y}, _step_y{step_y}
    , _from_t{from_t}, _to_t{to_t}, _step_t{step_t} {
    assert(_from_x <= _to_x && _from_y <= _to_y && _from_t <= _to_t);
    reset();
    // DEBUG
    std::cout << "My pose enumerator enabled\n";
  }

  bool has_next() const override {
    // Check if pose in rectangle
    if (failed_attemtps >= max_failed_attempts)
      return false;

    return _from_x <= _x && _x <= _to_x &&
           _from_y <= _y && _y <= _to_y;
  }

  RobotPose next(const RobotPose &prev_pose) override {
    if (!_base_pose_is_set) {
      _base_pose = prev_pose;
      _base_pose_is_set = true;
    }

    return {_base_pose.x + _x, _base_pose.y + _y, _base_pose.theta + _t};
  }

  void reset() override {
    _x = (_from_x + _to_x) / 2;
    _y = (_from_y + _to_y) / 2;
    _t = _from_t;
    
    number_of_steps = 1;
    stepped_times = 0;
    failed_attemtps = 0;
    direction = dir::left;
  }


  // Rotate for all angles in every cell
  void feedback(bool pose_is_acceptable) override {
    if (pose_is_acceptable) {
      failed_attemtps = 0;
    }
    else {
      failed_attemtps += 1;
    }

    if (_t <= _to_t - _step_t) {
      _t += _step_t;
      return;
    }

    _t = _from_t;

    if (stepped_times == number_of_steps) {
      
      stepped_times = 0;
      
      // Change direction
      switch(direction) {
        case dir::left:
          direction = dir::top;
          break;
        case dir::top:
          direction = dir::right;
          number_of_steps += 1;
          break;
        case dir::right:
          direction = dir::bot;
          break;
        case dir::bot:
          direction = dir::left;
          number_of_steps += 1;
          break;
      }
    }

    switch(direction) {
      case dir::left:
        _x -= _step_x;
        break;
      case dir::top:
        _y += _step_y;
        break;
      case dir::right:
        _x += _step_x;
        break;
      case dir::bot:
        _y -= _step_y;
        break;
    }

    stepped_times += 1;
  }

private:
  // TODO: use std::optional when C++17 is available
  bool _base_pose_is_set;
  RobotPose _base_pose;

  double _from_x, _x, _to_x, _step_x;
  double _from_y, _y, _to_y, _step_y;
  double _from_t, _t, _to_t, _step_t;

  long number_of_steps;
  long stepped_times;
  enum class dir {left, top, right, bot} direction;

  long failed_attemtps;
  const long max_failed_attempts = 50;
};

// TODO: add a PoseEnumerationScanMatcher descendant

class BruteForceScanMatcher : public PoseEnumerationScanMatcher {
public:
  BruteForceScanMatcher(std::shared_ptr<ScanProbabilityEstimator> estimator,
                        double from_x, double to_x, double step_x,
                        double from_y, double to_y, double step_y,
                        double from_t, double to_t, double step_t)
    : PoseEnumerationScanMatcher{
        estimator,
        std::make_shared<BruteForcePoseEnumerator>(from_x, to_x, step_x,
                                                   from_y, to_y, step_y,
                                                   from_t, to_t, step_t)
      } {}
};

#endif
