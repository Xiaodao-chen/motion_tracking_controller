//
// Created by qiayuanl on 3/7/25.
//

#pragma once

#include <legged_rl_controllers/ObservationManager.h>

#include "motion_tracking_controller/MotionCommand.h"
#include "motion_tracking_controller/common.h"
#include <deque>

namespace legged {

class MotionObservation : public ObservationTerm {
 public:
  // historySize: how many past values (including current) to keep.
  // Default to 1 (no history). Set to 10 to keep last 10 steps.
  explicit MotionObservation(const MotionCommandTerm::SharedPtr& commandTerm, size_t historySize = 1)
      : commandTerm_(commandTerm), historySize_(historySize) {}

  // clear stored history
  void clearHistory() { history_.clear(); }

  // reset hook called by manager when needed
  void reset() override { history_.clear(); }

  // ObservationTerm API: evaluate returns concatenated history vector
  vector_t evaluate() override {
    vector_t cur = evaluateCurrent();
    // push back current (we keep newest at back)
    history_.push_back(cur);
    if (history_.size() > historySize_) history_.pop_front();

    // concatenate history (oldest first -> newest last)
    const size_t base = cur.size();
    vector_t out = vector_t::Zero(base * history_.size());
    for (size_t i = 0; i < history_.size(); ++i) {
      out.segment(i * base, base) = history_[i];
    }
    return out;
  }

 protected:
  MotionCommandTerm::SharedPtr commandTerm_;
  std::deque<vector_t> history_;
  size_t historySize_ = 1;

  // Derived classes must implement this to return the current (single-step) observation
  virtual vector_t evaluateCurrent() = 0;
};

class MotionAnchorPosition final : public MotionObservation {
 public:
  using MotionObservation::MotionObservation;
  size_t getSize() const override { return 3 ; }

 protected:
  vector_t evaluateCurrent() override { return commandTerm_->getAnchorPositionLocal(); }
};

class MotionAnchorOrientation final : public MotionObservation {
 public:
  using MotionObservation::MotionObservation;
  size_t getSize() const override { return 6; }

 protected:
  vector_t evaluateCurrent() override { return commandTerm_->getAnchorOrientationLocal(); }
};

class RobotBodyPosition final : public MotionObservation {
 public:
  using MotionObservation::MotionObservation;
  size_t getSize() const override { return 3 * commandTerm_->getCfg().bodyNames.size() * historySize_; }

 protected:
  vector_t evaluateCurrent() override { return commandTerm_->getRobotBodyPositionLocal(); }
};

class RobotBodyOrientation final : public MotionObservation {
 public:
  using MotionObservation::MotionObservation;
  size_t getSize() const override { return 6 * commandTerm_->getCfg().bodyNames.size() * historySize_; }

 protected:
  vector_t evaluateCurrent() override { return commandTerm_->getRobotBodyOrientationLocal(); }
};

}  // namespace legged
