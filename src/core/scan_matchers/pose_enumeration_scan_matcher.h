#ifndef SLAM_CTOR_CORE_POSE_ENUMERATION_SCAN_MATCHER_H
#define SLAM_CTOR_CORE_POSE_ENUMERATION_SCAN_MATCHER_H

#include <functional>
#include <memory>
#include <cstdlib>
#include <chrono>

#include "pose_enumerators.h"
#include "grid_scan_matcher.h"

//#define DEBUG
#ifdef DEBUG
#include <iostream>
#endif

// TODO: merge the logic with hill climbing scan matcher
//       create free functions that create scan matchers
// TODO: move publish transform to observer
class PoseEnumerationScanMatcher : public GridScanMatcher {
public:
  PoseEnumerationScanMatcher(std::shared_ptr<ScanProbabilityEstimator> spe,
                             std::shared_ptr<PoseEnumerator> pe)
    : GridScanMatcher{spe}, _pose_enumerator{pe} {
      std::srand(0);
    }

  void set_pose_enumerator(std::shared_ptr<PoseEnumerator> pe) {
    _pose_enumerator = pe;
  }

  auto pose_enumerator() const { return _pose_enumerator; }

  // GridScanMatcher API implementation

  void reset_state() override {
    _pose_enumerator->reset();
  }

  double process_scan(const TransformedLaserScan &raw_scan,
                      const RobotPose &init_pose,
                      const GridMap &map,
                      RobotPoseDelta &pose_delta) override {

    do_for_each_observer([&init_pose, &raw_scan, &map](ObsPtr obs) {
      obs->on_matching_start(init_pose, raw_scan, map);
    });

    auto noised_pose = init_pose;
    //std::cout << "Pose : x = " << noised_pose.x << " , y = " << noised_pose.y << " , theta = " << noised_pose.theta << "\n";
    // static long poses = 0;
    
    // if (poses > 835) {
    //   double noise_x = (double(std::rand() % 50)) / 100;
    //   double noise_y = (double(std::rand() % 50)) / 100;
    //   //double noise_ = std::rand() % 1000;
    //   if (noise_x > noise_y) {
    //     noised_pose.x += noise_x;
    //     std::cout << "x " << noise_x << "\n";
    //   } else {
    //     noised_pose.y += noise_y;
    //     std::cout << "y " << noise_y << "\n";
    //   }
    // } else {
    //   poses++;
    // }

    static bool start_noising = false;
    if (noised_pose.x < -6.1)
      start_noising = true;

    if (start_noising) {
      double noise_x = (double(std::rand() % 100)) / 100;
      double noise_y = (double(std::rand() % 100)) / 100;  

      // if (noise_x > noise_y) {
      //   noised_pose.x += noise_x;
      //   std::cout << "x " << noise_x << "\n";
      // } else {
      //   noised_pose.y += noise_y;
      //   std::cout << "y " << noise_y << "\n";
      // }
      noised_pose.x += noise_x;
      noised_pose.y += noise_y;
    }
    
    auto scan = filter_scan(raw_scan.scan, noised_pose, map);

    auto best_pose = noised_pose;
    double lowest_scan_loss = scan_loss(scan, best_pose, map);

    do_for_each_observer([best_pose, scan, lowest_scan_loss](ObsPtr obs) {
      obs->on_scan_test(best_pose, scan, lowest_scan_loss);
      obs->on_pose_update(best_pose, scan, lowest_scan_loss);
    });

    _pose_enumerator->reset();
    auto begin = std::chrono::steady_clock::now();
    while (_pose_enumerator->has_next()) {

      auto sampled_pose = _pose_enumerator->next(best_pose);

      double sampled_scan_loss = scan_loss(scan, sampled_pose, map, lowest_scan_loss);
      
      do_for_each_observer([&sampled_pose, &scan,
                            &sampled_scan_loss](ObsPtr obs) {
        obs->on_scan_test(sampled_pose, scan, sampled_scan_loss);
      });

      auto pose_is_acceptable = lowest_scan_loss > sampled_scan_loss;

      _pose_enumerator->feedback(pose_is_acceptable);
      
      if (!pose_is_acceptable) {
        continue;
      }

      // update pose
      lowest_scan_loss = sampled_scan_loss;
      best_pose = sampled_pose;

      // notify pose update
      do_for_each_observer([&best_pose, &scan, &lowest_scan_loss](ObsPtr obs) {
        obs->on_pose_update(best_pose, scan, lowest_scan_loss);
      });
    }
    auto end = std::chrono::steady_clock::now();
    std::cout << std::chrono::duration_cast<std::chrono::milliseconds>(end - begin).count() << "\n";
    pose_delta = best_pose - init_pose;

    //#ifdef DEBUG
    //std::cout << "Pose delta: x = " << pose_delta.x << " , y = " << pose_delta.y << " , theta = " << pose_delta.theta << "\n";
    //#endif
    
    do_for_each_observer([&scan, &pose_delta, &lowest_scan_loss](ObsPtr obs) {
        obs->on_matching_end(pose_delta, scan, lowest_scan_loss);
    });

    return lowest_scan_loss;
  }

private:
  std::shared_ptr<PoseEnumerator> _pose_enumerator;

};

#endif