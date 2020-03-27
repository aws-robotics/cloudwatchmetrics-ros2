/*
 * Copyright 2020 Amazon.com, Inc. or its affiliates. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License").
 * You may not use this file except in compliance with the License.
 * A copy of the License is located at
 *
 *  http://aws.amazon.com/apache2.0
 *
 * or in the "license" file accompanying this file. This file is distributed
 * on an "AS IS" BASIS, WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either
 * express or implied. See the License for the specific language governing
 * permissions and limitations under the License.
 */

#pragma once

#include <chrono>
#include <limits>

#include <builtin_interfaces/msg/time.hpp>
#include <ros_monitoring_msgs/msg/metric_data.hpp>

constexpr char kMeasurementSource[] = "CWMetricsNodeTest";
constexpr char kMetricName[] = "CWMetricsNodeTestMetric";
constexpr char kMetricUnit[] = "sec";

template<typename chrono_time_point>
void chrono_time_point_to_builtin_interfaces_time(
  const chrono_time_point & timepoint,
  builtin_interfaces::msg::Time & timestamp)
{
  auto secs = std::chrono::time_point_cast<std::chrono::seconds>(timepoint);
  auto nanosecs = std::chrono::time_point_cast<std::chrono::nanoseconds>(timepoint)
                  - std::chrono::time_point_cast<std::chrono::nanoseconds>(secs);
  timestamp.sec = secs.time_since_epoch().count();
  timestamp.nanosec = nanosecs.count();
}

inline ros_monitoring_msgs::msg::MetricData EmptyMonitoringData()
{
  ros_monitoring_msgs::msg::MetricData data = ros_monitoring_msgs::msg::MetricData();
  data.metric_name = kMetricName;
  data.unit = kMetricUnit;
  data.value = std::numeric_limits<decltype(data.value)>::quiet_NaN();
  chrono_time_point_to_builtin_interfaces_time(std::chrono::system_clock::now(), data.time_stamp);
  return data;
}
