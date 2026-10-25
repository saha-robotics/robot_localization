/*
 * Copyright (c) 2014, 2015, 2016 Charles River Analytics, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 * notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above
 * copyright notice, this list of conditions and the following
 * disclaimer in the documentation and/or other materials provided
 * with the distribution.
 * 3. Neither the name of the copyright holder nor the names of its
 * contributors may be used to endorse or promote products derived
 * from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include <gtest/gtest.h>

#include <cmath>
#include <chrono>
#include <limits>
#include <vector>
#include <memory>
#include <stdexcept>
#include <thread>

#include <robot_localization/filter_base.hpp>
#include "robot_localization/ekf.hpp"
#include "robot_localization/ros_filter.hpp"
#include "robot_localization/ros_filter_types.hpp"

using robot_localization::Ekf;
using robot_localization::RosEkf;
using robot_localization::STATE_SIZE;

TEST(EkfTest, Measurements) {
  // node handle is created as per ros2
  rclcpp::NodeOptions options;
  options.arguments({"ekf_filter_node"});
  std::shared_ptr<robot_localization::RosEkf> filter =
    std::make_shared<robot_localization::RosEkf>(options);
  filter->initialize();

  // create the instance of the class and pass parameters
  Eigen::MatrixXd initialCovar(15, 15);

  initialCovar.setIdentity();
  initialCovar *= 0.5;

  filter->getFilter().setEstimateErrorCovariance(initialCovar);

  Eigen::VectorXd measurement(STATE_SIZE);
  measurement.setIdentity();

  for (size_t i = 0; i < STATE_SIZE; ++i) {
    measurement[i] = i * 0.01 * STATE_SIZE;
  }
  Eigen::MatrixXd measurementCovariance(STATE_SIZE, STATE_SIZE);
  measurementCovariance.setIdentity();
  for (size_t i = 0; i < STATE_SIZE; ++i) {
    measurementCovariance(i, i) = 1e-9;
  }
  std::vector<bool> updateVector(STATE_SIZE, true);

  // Ensure that measurements are being placed in the queue correctly
  rclcpp::Time time1(1000);
  filter->robot_localization::RosEkf::enqueueMeasurement(
    "odom0", measurement, measurementCovariance, updateVector,
    std::numeric_limits<double>::max(), time1);

  filter->robot_localization::RosEkf::integrateMeasurements(rclcpp::Time(1001));

  EXPECT_EQ(filter->getFilter().getState(), measurement);
  EXPECT_EQ(
    filter->getFilter().getEstimateErrorCovariance(),
    measurementCovariance);

  filter->getFilter().setEstimateErrorCovariance(initialCovar);

  // Now fuse another measurement and check the output.
  // We know what the filter's state should be when
  // this is complete, so we'll check the difference and
  // make sure it's suitably small.
  Eigen::VectorXd measurement2 = measurement;

  measurement2 *= 2.0;

  for (size_t i = 0; i < STATE_SIZE; ++i) {
    measurementCovariance(i, i) = 1e-9;
  }

  rclcpp::Time time2(1002);

  filter->robot_localization::RosEkf::enqueueMeasurement(
    "odom0", measurement2, measurementCovariance, updateVector,
    std::numeric_limits<double>::max(), time2);

  filter->robot_localization::RosEkf::integrateMeasurements(rclcpp::Time(1003));

  measurement = measurement2.eval() - filter->getFilter().getState();
  for (size_t i = 0; i < STATE_SIZE; ++i) {
    EXPECT_LT(::fabs(measurement[i]), 0.001);
  }
}

namespace
{
class HistoryFilter : public RosEkf
{
public:
  explicit HistoryFilter(bool reject_out_of_history = true)
  : RosEkf(rclcpp::NodeOptions().arguments({"history_filter_test"}))
  {
    smooth_lagged_data_ = true;
    reject_out_of_history_measurements_ = reject_out_of_history;
    history_length_ = rclcpp::Duration::from_seconds(2.0);
    waitForTfListener();
  }

  explicit HistoryFilter(const rclcpp::NodeOptions & options)
  : RosEkf(options)
  {
    waitForTfListener();
  }

  static rclcpp::Time at(const int64_t ns)
  {
    return rclcpp::Time(ns, RCL_ROS_TIME);
  }

  void add(const int64_t ns, const double yaw)
  {
    Eigen::VectorXd measurement = Eigen::VectorXd::Zero(STATE_SIZE);
    measurement(robot_localization::StateMemberYaw) = yaw;
    measurement(robot_localization::StateMemberVyaw) = 1.0;
    enqueueMeasurement(
      "pose", measurement, Eigen::MatrixXd::Identity(STATE_SIZE, STATE_SIZE) * 0.01,
      std::vector<bool>(STATE_SIZE, true), std::numeric_limits<double>::max(), at(ns));
  }

  void fuse(const int64_t ns, const double yaw)
  {
    add(ns, yaw);
    integrateMeasurements(at(ns));
  }

  void expire(const int64_t ns) {clearExpiredHistory(at(ns));}
  size_t states() const {return filter_state_history_.size();}
  size_t measurements() const {return measurement_history_.size();}
  size_t queued() const {return measurement_queue_.size();}
  bool historyWarning() const {return dynamic_diagnostics_.count("pose_history") != 0;}
  void disableSmoothing() {smooth_lagged_data_ = false;}
  bool guardEnabled() const {return reject_out_of_history_measurements_;}

private:
  void waitForTfListener()
  {
    // Humble listener teardown can cancel before its new thread starts spinning.
    auto publisher = create_publisher<tf2_msgs::msg::TFMessage>(
      "/tf_static", rclcpp::QoS(1).transient_local());
    tf2_msgs::msg::TFMessage message;
    geometry_msgs::msg::TransformStamped transform;
    transform.header.frame_id = "history_fixture_parent";
    transform.child_frame_id = "history_fixture_child";
    transform.transform.rotation.w = 1.0;
    message.transforms.push_back(transform);
    publisher->publish(message);
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(5);
    while (!tf_buffer_->canTransform(
        transform.header.frame_id, transform.child_frame_id, tf2::TimePointZero))
    {
      if (std::chrono::steady_clock::now() >= deadline) {
        throw std::runtime_error("TF listener did not receive the fixture readiness transform");
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
  }
};

void expectSameEstimate(HistoryFilter & a, HistoryFilter & b)
{
  EXPECT_TRUE(a.getFilter().getState().isApprox(b.getFilter().getState(), 1e-10));
  EXPECT_TRUE(a.getFilter().getEstimateErrorCovariance().isApprox(
      b.getFilter().getEstimateErrorCovariance(), 1e-10));
  EXPECT_EQ(a.getFilter().getLastMeasurementTime(), b.getFilter().getLastMeasurementTime());
}
}  // namespace

TEST(EkfHistoryTest, CoveredLateMeasurementMatchesChronologicalOrder)
{
  HistoryFilter chronological;
  chronological.fuse(10000000000LL, 0.0);
  chronological.fuse(10500000000LL, 0.4);
  chronological.fuse(11000000000LL, 1.0);

  HistoryFilter delayed;
  delayed.fuse(10000000000LL, 0.0);
  delayed.fuse(11000000000LL, 1.0);
  delayed.add(10500000000LL, 0.4);
  delayed.integrateMeasurements(HistoryFilter::at(11000000000LL));
  expectSameEstimate(chronological, delayed);
  EXPECT_FALSE(delayed.historyWarning());
}

TEST(EkfHistoryTest, CoveredDelaysUseTheirMeasurementTime)
{
  for (const int64_t delay : {20000000LL, 500000000LL, 1080000000LL, 1999999999LL}) {
    SCOPED_TRACE(delay);
    const int64_t end = 12000000000LL;
    HistoryFilter ordered;
    ordered.fuse(10000000000LL, 0.0);
    ordered.fuse(end - delay, 0.4);
    ordered.fuse(end, 1.0);
    HistoryFilter delayed;
    delayed.fuse(10000000000LL, 0.0);
    delayed.fuse(end, 1.0);
    delayed.add(end - delay, 0.4);
    delayed.integrateMeasurements(HistoryFilter::at(end));
    expectSameEstimate(ordered, delayed);
    EXPECT_FALSE(delayed.historyWarning());
  }
}

TEST(EkfHistoryTest, ExpiredBurstDoesNotConsumeFutureMeasurement)
{
  HistoryFilter filter;
  HistoryFilter reference;
  for (auto * f : {&filter, &reference}) {
    f->fuse(10000000000LL, 0.0);
    f->fuse(11000000000LL, 1.0);
  }
  filter.add(6000000000LL, -1.0);
  filter.add(5000000000LL, -2.0);
  filter.add(12000000000LL, 2.0);
  filter.integrateMeasurements(HistoryFilter::at(11000000000LL));
  expectSameEstimate(filter, reference);
  EXPECT_EQ(filter.queued(), 1u);
  EXPECT_TRUE(filter.historyWarning());
  filter.integrateMeasurements(HistoryFilter::at(12000000000LL));
  reference.fuse(12000000000LL, 2.0);
  expectSameEstimate(filter, reference);
  EXPECT_EQ(filter.queued(), 0u);
}

TEST(EkfHistoryTest, ExpiredAndEqualBoundaryDoNotChangeStateOrBlockNewInput)
{
  HistoryFilter filter;
  HistoryFilter reference;
  for (auto * f : {&filter, &reference}) {
    f->fuse(10000000000LL, 0.0);
    f->fuse(11000000000LL, 1.0);
    f->fuse(12000000000LL, 2.0);
    f->expire(10500000000LL);
  }
  const auto state_count = filter.states();
  const auto measurement_count = filter.measurements();
  filter.add(10500000000LL, -1.0);
  filter.add(11000000000LL, -1.0);
  filter.integrateMeasurements(HistoryFilter::at(12000000000LL));
  expectSameEstimate(filter, reference);
  EXPECT_EQ(filter.states(), state_count);
  EXPECT_EQ(filter.measurements(), measurement_count);
  EXPECT_TRUE(filter.historyWarning());

  filter.add(10500000000LL, -1.0);
  filter.add(12200000000LL, 2.1);
  filter.integrateMeasurements(HistoryFilter::at(12200000000LL));
  reference.fuse(12200000000LL, 2.1);
  expectSameEstimate(filter, reference);
}

TEST(EkfHistoryTest, BackwardClockDefersFutureSampleBeforeCheckingHistory)
{
  for (const int64_t stamp : {10500000000LL, 11500000000LL}) {
    SCOPED_TRACE(stamp);
    HistoryFilter filter;
    HistoryFilter reference;
    for (auto * f : {&filter, &reference}) {
      f->fuse(10000000000LL, 0.0);
      f->fuse(11000000000LL, 1.0);
      f->fuse(12000000000LL, 2.0);
      f->expire(10500000000LL);
    }
    const auto state_count = filter.states();
    const auto measurement_count = filter.measurements();
    filter.add(stamp, -1.0);
    filter.integrateMeasurements(HistoryFilter::at(10000000000LL));
    expectSameEstimate(filter, reference);
    EXPECT_EQ(filter.states(), state_count);
    EXPECT_EQ(filter.measurements(), measurement_count);
    EXPECT_EQ(filter.queued(), 1u);
    EXPECT_FALSE(filter.historyWarning());

    filter.integrateMeasurements(HistoryFilter::at(12000000000LL));
    EXPECT_EQ(filter.queued(), 0u);
    if (stamp == 10500000000LL) {
      expectSameEstimate(filter, reference);
      EXPECT_TRUE(filter.historyWarning());
    } else {
      reference.add(stamp, -1.0);
      reference.integrateMeasurements(HistoryFilter::at(12000000000LL));
      expectSameEstimate(filter, reference);
      EXPECT_FALSE(filter.historyWarning());
    }
  }
}

TEST(EkfHistoryTest, EmptyHistoryRejectsLateInputButResetCanInitialize)
{
  HistoryFilter filter;
  filter.fuse(10000000000LL, 0.0);
  filter.fuse(11000000000LL, 1.0);
  filter.expire(12000000000LL);
  ASSERT_EQ(filter.states(), 0u);
  const Eigen::VectorXd state = filter.getFilter().getState();
  const Eigen::MatrixXd covariance = filter.getFilter().getEstimateErrorCovariance();
  filter.add(10500000000LL, -1.0);
  filter.integrateMeasurements(HistoryFilter::at(11000000000LL));
  EXPECT_EQ(filter.getFilter().getState(), state);
  EXPECT_EQ(filter.getFilter().getEstimateErrorCovariance(), covariance);
  EXPECT_EQ(filter.states(), 0u);
  EXPECT_EQ(filter.measurements(), 0u);
  EXPECT_TRUE(filter.historyWarning());

  filter.reset();
  ASSERT_FALSE(filter.getFilter().getInitializedStatus());
  filter.fuse(1000000000LL, 0.5);
  EXPECT_TRUE(filter.getFilter().getInitializedStatus());
  EXPECT_DOUBLE_EQ(filter.getFilter().getState()(robot_localization::StateMemberYaw), 0.5);
  EXPECT_EQ(filter.states(), 1u);
}

TEST(EkfHistoryTest, DisabledSmoothingKeepsExistingBehavior)
{
  HistoryFilter filter;
  filter.disableSmoothing();
  filter.fuse(10000000000LL, 0.0);
  filter.fuse(11000000000LL, 1.0);
  const double yaw = filter.getFilter().getState()(robot_localization::StateMemberYaw);
  filter.add(10500000000LL, -1.0);
  filter.integrateMeasurements(HistoryFilter::at(11000000000LL));
  EXPECT_LT(filter.getFilter().getState()(robot_localization::StateMemberYaw), yaw);
  EXPECT_FALSE(filter.historyWarning());
}

TEST(EkfHistoryTest, HistoryGuardIsOptIn)
{
  rclcpp::NodeOptions options;
  options.arguments({"history_filter_test"});
  auto node = std::make_shared<HistoryFilter>(options);
  node->initialize();
  auto & filter = *node;
  EXPECT_FALSE(filter.get_parameter("reject_out_of_history_measurements").as_bool());
  EXPECT_FALSE(filter.guardEnabled());
}

TEST(EkfHistoryTest, SetPosePreservedUntilCurrentMeasurementRebuildsHistory)
{
  rclcpp::NodeOptions options;
  options.arguments({"history_filter_test"});
  options.parameter_overrides({
    rclcpp::Parameter("smooth_lagged_data", true),
    rclcpp::Parameter("history_length", 2.0),
    rclcpp::Parameter("reject_out_of_history_measurements", true)});
  auto node = std::make_shared<HistoryFilter>(options);
  node->initialize();
  auto & filter = *node;
  ASSERT_TRUE(filter.guardEnabled());
  const auto before = filter.now().nanoseconds();
  filter.fuse(before - 1000000000LL, 0.0);
  filter.fuse(before - 500000000LL, 1.0);
  filter.add(before + 1000000000LL, -1.0);
  auto pose = std::make_shared<geometry_msgs::msg::PoseWithCovarianceStamped>();
  pose->header.frame_id = "odom";
  pose->header.stamp = filter.now();
  pose->pose.pose.position.x = 3.0;
  pose->pose.pose.orientation.z = std::sin(0.4);
  pose->pose.pose.orientation.w = std::cos(0.4);
  for (size_t i = 0; i < 36; i += 7) {pose->pose.covariance[i] = 0.01;}
  filter.setPoseCallback(pose);
  ASSERT_TRUE(filter.getFilter().getInitializedStatus());
  EXPECT_EQ(filter.queued(), 0u);
  EXPECT_EQ(filter.states(), 0u);
  EXPECT_EQ(filter.measurements(), 0u);
  const auto reset_stamp = filter.getFilter().getLastMeasurementTime().nanoseconds();
  const Eigen::VectorXd state = filter.getFilter().getState();
  const Eigen::MatrixXd covariance = filter.getFilter().getEstimateErrorCovariance();
  EXPECT_NEAR(state(robot_localization::StateMemberX), 3.0, 1e-10);
  EXPECT_NEAR(state(robot_localization::StateMemberYaw), 0.8, 1e-10);
  filter.add(reset_stamp - 1, -1.0);
  filter.integrateMeasurements(HistoryFilter::at(reset_stamp));
  EXPECT_EQ(filter.getFilter().getState(), state);
  EXPECT_EQ(filter.getFilter().getEstimateErrorCovariance(), covariance);
  EXPECT_TRUE(filter.historyWarning());
  EXPECT_EQ(filter.states(), 0u);
  filter.fuse(reset_stamp + 20000000LL, 0.81);
  EXPECT_EQ(filter.states(), 1u);
  EXPECT_EQ(filter.queued(), 0u);
  nav_msgs::msg::Odometry output;
  ASSERT_TRUE(filter.getFilteredOdometryMessage(&output));
  EXPECT_EQ(rclcpp::Time(output.header.stamp).nanoseconds(), reset_stamp + 20000000LL);
  EXPECT_TRUE(filter.getFilter().getEstimateErrorCovariance().allFinite());
}

TEST(EkfHistoryTest, DisabledGuardKeepsExistingPartialRewindBehavior)
{
  HistoryFilter filter(false);
  filter.fuse(10000000000LL, 0.0);
  filter.fuse(11000000000LL, 1.0);
  filter.fuse(12000000000LL, 2.0);
  filter.expire(10500000000LL);
  const Eigen::VectorXd before = filter.getFilter().getState();
  filter.add(10500000000LL, -1.0);
  filter.integrateMeasurements(HistoryFilter::at(12000000000LL));
  EXPECT_FALSE(filter.getFilter().getState().isApprox(before, 1e-10));
  EXPECT_FALSE(filter.historyWarning());
}

TEST(EkfPublicationTest, SuppressedEstimatesDoNotLowerTheLastPublishedStamp)
{
  for (const bool permit_corrections : {false, true}) {
    SCOPED_TRACE(permit_corrections);
    rclcpp::NodeOptions options;
    options.arguments({"publication_filter_test"});
    options.parameter_overrides({
      rclcpp::Parameter("permit_corrected_publication", permit_corrections),
      rclcpp::Parameter("sensor_timeout", 3600.0),
      rclcpp::Parameter("predict_to_current_time", false),
      rclcpp::Parameter("publish_tf", true),
      rclcpp::Parameter("publish_acceleration", true),
      rclcpp::Parameter("transform_time_offset", 0.0),
      rclcpp::Parameter("world_frame", "odom")});
    auto filter = std::make_shared<HistoryFilter>(options);
    filter->initialize();
    auto observer = std::make_shared<rclcpp::Node>("publication_observer");
    std::vector<int64_t> odom_stamps, tf_stamps, accel_stamps;
    auto odom = observer->create_subscription<nav_msgs::msg::Odometry>(
      "odometry/filtered", 10, [&](nav_msgs::msg::Odometry::ConstSharedPtr msg) {
        odom_stamps.push_back(rclcpp::Time(msg->header.stamp).nanoseconds());
      });
    auto tf = observer->create_subscription<tf2_msgs::msg::TFMessage>(
      "/tf", 10, [&](tf2_msgs::msg::TFMessage::ConstSharedPtr msg) {
        for (const auto & transform : msg->transforms) {
          if (transform.header.frame_id == "odom" && transform.child_frame_id == "base_link") {
            tf_stamps.push_back(rclcpp::Time(transform.header.stamp).nanoseconds());
          }
        }
      });
    auto accel = observer->create_subscription<geometry_msgs::msg::AccelWithCovarianceStamped>(
      "accel/filtered", 10,
      [&](geometry_msgs::msg::AccelWithCovarianceStamped::ConstSharedPtr msg) {
        accel_stamps.push_back(rclcpp::Time(msg->header.stamp).nanoseconds());
      });
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(observer);
    const auto wait_until = [&](const auto & ready) {
        const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(5);
        while (!ready() && std::chrono::steady_clock::now() < deadline) {
          executor.spin_some();
          std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
        return ready();
      };
    ASSERT_TRUE(wait_until([&]() {
      return odom->get_publisher_count() == 1 && tf->get_publisher_count() == 1 &&
             accel->get_publisher_count() == 1;
    }));
    const auto received_through = [&](const int64_t stamp) {
        return wait_until([&]() {
          return !odom_stamps.empty() && odom_stamps.back() == stamp &&
                 !tf_stamps.empty() && tf_stamps.back() == stamp &&
                 !accel_stamps.empty() && accel_stamps.back() == stamp;
        });
      };
    const int64_t base = filter->now().nanoseconds() - 2000000000LL;
    filter->fuse(base + 1000000000LL, 0.0);
    // Inject estimate times to isolate publication, not the history-rewind algorithm.
    // Only the observer spins; the filter's timer cannot add another update.
    for (const int64_t offset : {1000000000LL, 0LL, 100000000LL, 1000000000LL, 1100000000LL}) {
      filter->getFilter().setLastMeasurementTime(HistoryFilter::at(base + offset));
      filter->periodicUpdate();
    }
    ASSERT_TRUE(received_through(base + 1100000000LL));
    const std::vector<int64_t> expected = permit_corrections ?
      std::vector<int64_t>{base + 1000000000LL, base, base + 100000000LL,
        base + 1000000000LL, base + 1100000000LL} :
      std::vector<int64_t>{base + 1000000000LL, base + 1100000000LL};
    EXPECT_EQ(odom_stamps, expected);
    EXPECT_EQ(tf_stamps, expected);
    EXPECT_EQ(accel_stamps, expected);

    odom_stamps.clear();
    tf_stamps.clear();
    accel_stamps.clear();
    filter->reset();
    // FilterBase::reset restores its default timeout; keep this publication test frozen.
    filter->getFilter().setSensorTimeout(rclcpp::Duration::from_seconds(3600.0));
    filter->fuse(base, 0.0);
    filter->periodicUpdate();
    filter->fuse(base + 100000000LL, 0.1);
    filter->periodicUpdate();
    ASSERT_TRUE(received_through(base + 100000000LL));
    const std::vector<int64_t> reset_expected{base, base + 100000000LL};
    EXPECT_EQ(odom_stamps, reset_expected);
    EXPECT_EQ(tf_stamps, reset_expected);
    EXPECT_EQ(accel_stamps, reset_expected);
  }
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  ::testing::InitGoogleTest(&argc, argv);
  int ret = RUN_ALL_TESTS();
  rclcpp::shutdown();

  return ret;
}
