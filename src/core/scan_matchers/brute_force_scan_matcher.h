#ifndef SLAM_CTOR_CORE_BRUTE_FORCE_SCAN_MATCHER_H
#define SLAM_CTOR_CORE_BRUTE_FORCE_SCAN_MATCHER_H

#include <cassert>
#include <chrono>
#include <thread>

#include "pose_enumeration_scan_matcher.h"

//#define DEBUG

// TODO: moved_current_direction to pose enumerators
// FIXME: class name
class BruteForcePoseEnumerator : public PoseEnumerator {
public:
  BruteForcePoseEnumerator(double from_x, double to_x, double step_x,
                           double from_y, double to_y, double step_y,
                           double from_t, double to_t, double step_t)
  {
    std::cout << "My brute-force pose enumerator enabled. " << max_attempts << " failed attempts\n";
  } 

  bool has_next() const override {
    return attempts < max_attempts;
  }

  RobotPose next(const RobotPose &prev_pose) override {
    if (!_base_pose_is_set) {
      _current_pose = prev_pose; 
      _base_pose_is_set = true;
      _current_pose.theta -= _step_t * (max_angle_attempts / 2);

      _from_t = _current_pose.theta;
      
      #ifdef DEBUG
      std::cout << "\n\nBase pose was set x = " << _current_pose.x << " , y = " << _current_pose.y << " , theta = " << _current_pose.theta << "\n";
      #endif
    }

    if (angle_attempts < max_angle_attempts) {
      _current_pose.theta += _step_t;
      angle_attempts += 1;
    } else {
      _current_pose.theta = _from_t;
      angle_attempts = 0;

      #ifdef DEBUG
      std::chrono::seconds dura(1);
      std::this_thread::sleep_for( dura );
      std::cout << "Current moved_current_direction " << moved_current_direction << " Have to moved_current_direction " << should_move_current_direction << " Direction = " << direction << "\n";
      #endif

      if (moved_current_direction == should_move_current_direction) {
        moved_current_direction = 0;
        update_direction();

        #ifdef DEBUG
        std::cout << "Direction was updated " << direction << "\n";
        #endif
      }

      move();
    }

    #ifdef DEBUG
    std::cout << "Next pose x = " << _current_pose.x << " , y = " << _current_pose.y << " , theta = " << _current_pose.theta << "\n";
    #endif

    return _current_pose;
  }

  void update_direction() {
    switch(direction) {
      case 0:
        direction += 1;
        break;
      case 1:
        direction += 1;
        should_move_current_direction += 1;
        break;
      case 2: 
        direction += 1;
        break;
      case 3:
        direction = 0;
        should_move_current_direction += 1;
        break;
    }
  }

  void move() {
    switch(direction) {
      case 0:
        _current_pose.x -= _step_x;
        break;
      case 1:
        _current_pose.y += _step_y;
        break;
      case 2:
        _current_pose.x += _step_x;
        break;
      case 3:
        _current_pose.y -= _step_y;
        break;
    }

    moved_current_direction += 1;
    attempts += 1;
  }

  void reset() override {
    
    #ifdef DEBUG
    std::chrono::seconds dura(1);
    std::this_thread::sleep_for( dura );
    #endif

    _base_pose_is_set = false;
    angle_attempts = 0;
    attempts = 0;

    should_move_current_direction = 1;
    moved_current_direction = 0;
    direction = 0;
  }

  void feedback(bool pose_is_acceptable) override {
    if (pose_is_acceptable) {
      if (attempts > max_attempts /  2)
        attempts = max_attempts /  2;
    } 
  }

private:
  // TODO: use std::optional when C++17 is available
  bool _base_pose_is_set = false;
  
  double _step_x = 0.02;
  double _step_y = 0.02;
  // TODO : fix _from_t
  double _from_t = -100000, _step_t = 0.0003;

  long max_angle_attempts = 20;
  long angle_attempts = 0;

  long max_attempts = 150;
  long attempts = 0;

  long should_move_current_direction = 1;
  long moved_current_direction = 0;
  short direction = 0;

  RobotPose _current_pose;
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