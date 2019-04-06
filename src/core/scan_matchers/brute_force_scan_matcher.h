#ifndef SLAM_CTOR_CORE_BRUTE_FORCE_SCAN_MATCHER_H
#define SLAM_CTOR_CORE_BRUTE_FORCE_SCAN_MATCHER_H

#include <cassert>
#include <algorithm>

#include "pose_enumeration_scan_matcher.h"

//#define DEBUG
#ifdef DEBUG
#include <thread>
#endif

// TODO: _moved_current_direction to pose enumerators
// FIXME: class name
class BruteForcePoseEnumerator : public PoseEnumerator {
public:
  BruteForcePoseEnumerator(long max_angle_attempts, long max_attempts, 
                           double step_x, double step_y, double step_t)
    : _max_angle_attempts(max_angle_attempts)
    , _max_attempts(max_attempts)
    , _step_x(step_x)
    , _step_y(step_y)
    , _step_t(step_t) {
    std::cout << "BF pose enumerator enabled: " << _max_attempts << " failed attempts "
              << _max_angle_attempts << " angle_attempts "  << _step_x << " shift x " 
              << _step_y << " shift y " << _step_t << " shift t\n";
  } 

  bool has_next() const override {
    return _attempts < _max_attempts;
  }

  RobotPose next(const RobotPose &prev_pose) override {
    if (!_first_pose_is_set) {
      _current_pose = prev_pose; 
      _first_pose_is_set = true;
      _current_pose.theta -= _step_t * (_max_angle_attempts / 2);

      _base_theta = _current_pose.theta;
      
      #ifdef DEBUG
      std::cout << "\n\nBase pose was set x = " << _current_pose.x << " , y = " << _current_pose.y << " , theta = " << _current_pose.theta << "\n";
      #endif
    }

    if (_angle_attempts < _max_angle_attempts) {
      _current_pose.theta += _step_t;
      _angle_attempts += 1;
    } else {
      _current_pose.theta = _base_theta;
      _angle_attempts = 0;

      #ifdef DEBUG
      std::chrono::seconds dura(1);
      std::this_thread::sleep_for( dura );
      std::cout << "Current _moved_current_direction " << _moved_current_direction << " Have to _moved_current_direction " << _current_direction_moves << " _direction = " << _direction << "\n";
      #endif

      if (_moved_current_direction == _current_direction_moves) {
        update__direction();
        _moved_current_direction = 0;

        #ifdef DEBUG
        std::cout << "_direction was updated " << _direction << "\n";
        #endif
      }

      move();
    }

    #ifdef DEBUG
    std::cout << "Next pose x = " << _current_pose.x << " , y = " << _current_pose.y << " , theta = " << _current_pose.theta << "\n";
    #endif

    return _current_pose;
  }

  void update__direction() {
    switch(_direction) {
      case 0:
        _direction += 1;
        break;
      case 1:
        _direction += 1;
        _current_direction_moves += 1;
        break;
      case 2: 
        _direction += 1;
        break;
      case 3:
        _direction = 0;
        _current_direction_moves += 1;
        break;
    }
  }

  void move() {
    // Moves left, up, right, down
    switch(_direction) {
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

    _moved_current_direction += 1;
    _attempts += 1;
  }

  void reset() override {
    
    #ifdef DEBUG
    std::chrono::seconds dura(1);
    std::this_thread::sleep_for( dura );
    #endif

    _first_pose_is_set = false;
    _angle_attempts = 0;
    _attempts = 0;

    _current_direction_moves = 1;
    _moved_current_direction = 0;
    _direction = 0;
  }

  void feedback(bool pose_is_acceptable) override {
    if (pose_is_acceptable) {
      _attempts = std::min(_attempts, _max_attempts / 2);
    } 
  }

private:
  // TODO: use std::optional when C++17 is available
  bool _first_pose_is_set = false;
  
  const long _max_angle_attempts;
  const long _max_attempts;

  long _angle_attempts = 0;
  long _attempts = 0;

  const double _step_x;
  const double _step_y;
  const double _step_t;

  // TODO : fix _base_theta
  double _base_theta = -100000;

  long _current_direction_moves = 1;
  long _moved_current_direction = 0;
  short _direction = 0;

  RobotPose _current_pose;
};

// TODO: add a PoseEnumerationScanMatcher descendant

class BruteForceScanMatcher : public PoseEnumerationScanMatcher {
public:
  BruteForceScanMatcher(std::shared_ptr<ScanProbabilityEstimator> estimator,
                        long max_angle_attempts, long max_attempts,
                        double step_x, double step_y, double step_t)
    : PoseEnumerationScanMatcher{
        estimator,
        std::make_shared<BruteForcePoseEnumerator>(max_angle_attempts, max_attempts,
                                                   step_x, step_y, step_t)
      } {}
};

#endif