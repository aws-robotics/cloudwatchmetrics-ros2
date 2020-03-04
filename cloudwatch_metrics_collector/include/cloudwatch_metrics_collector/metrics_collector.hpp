/*
 * Copyright 2019 Amazon.com, Inc. or its affiliates. All Rights Reserved.
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

#include <aws_common/sdk_utils/logging/aws_log_system.h>
#include <rclcpp/rclcpp.hpp>
#include <ros_monitoring_msgs/msg/metric_list.hpp>
#include <ros_monitoring_msgs/msg/metric_data.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <std_srvs/srv/empty.hpp>

#include <cloudwatch_metrics_common/metric_service.hpp>
#include <cloudwatch_metrics_common/metric_service_factory.hpp>

#include <string>
#include <map>

namespace Aws {
namespace CloudWatchMetrics {
namespace Utils {

class MetricsCollector : public Service
{
public:

  MetricsCollector() = default;
  ~MetricsCollector() = default;

  /**
   * Accept input metric message to be batched for publishing.
   *
   * @param metric_list_msg
   * @return the number of metrics successfully batched
   */
  int RecordMetrics(const ros_monitoring_msgs::msg::MetricList::UniquePtr metric_list_msg);

  /**
   * Force all batched data to be published to CloudWatch.
   */
  void TriggerPublish();

  /**
   * Initialize the MetricsCollector with parameters read from the config file.
   *
   * @param metric_namespace
   * @param default_dimensions
   * @param storage_resolution
   * @param config
   * @param sdk_options
   * @param metric_service_factory
   */
  void Initialize(std::string metric_namespace,
                  std::map<std::string, std::string> & default_dimensions,
                  int storage_resolution,
                  rclcpp::Node::SharedPtr node,
                  const Aws::Client::ClientConfiguration & config,
                  const Aws::SDKOptions & sdk_options,
                  const Aws::CloudWatchMetrics::CloudWatchOptions & cloudwatch_options,
                  const std::vector<std::string>&  topics,
                  std::shared_ptr<MetricServiceFactory> metric_service_factory = std::make_shared<MetricServiceFactory>());

  void SubscribeAllTopics();

  bool start() override;
  bool shutdown() override;

  /**
   * Return a Trigger response detailing the MetricService online status.
   *
   * @param request input request
   * @param response output response
   * @return true if the request was handled successfully, false otherwise
   */
  bool checkIfOnline(std::shared_ptr<std_srvs::srv::Trigger::Request> request, std::shared_ptr<std_srvs::srv::Trigger::Response> response);

  /**
   * Gets the timestamp for the input metric message as milliseconds since epoch
   */
  static int64_t GetMetricDataEpochMillis(const ros_monitoring_msgs::msg::MetricData & metric_msg);

private:

  std::string metric_namespace_;
  std::map<std::string, std::string> default_dimensions_;
  std::atomic<int> storage_resolution_;
  std::shared_ptr<MetricService> metric_service_;
  std::vector<std::shared_ptr<rclcpp::Subscription<ros_monitoring_msgs::msg::MetricList>>> subscriptions_;
  rclcpp::Node::SharedPtr node_;
  std::vector<std::string> topics_;
};

}  // namespace Utils
}  // namespace CloudWatchMetrics
}  // namespace Aws
