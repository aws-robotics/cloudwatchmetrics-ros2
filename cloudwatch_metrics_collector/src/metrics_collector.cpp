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

#include <aws/core/Aws.h>
#include <aws/core/client/ClientConfiguration.h>
#include <aws/core/utils/StringUtils.h>
#include <aws/core/utils/logging/AWSLogging.h>
#include <aws/core/utils/logging/LogMacros.h>
#include <aws/monitoring/CloudWatchClient.h>
#include <aws/monitoring/model/PutMetricDataRequest.h>
#include <aws_common/sdk_utils/client_configuration_provider.h>
#include <aws_ros2_common/sdk_utils/logging/aws_ros_logger.h>
#include <aws_ros2_common/sdk_utils/ros2_node_parameter_reader.h>
#include <rclcpp/rclcpp.hpp>
#include <ros_monitoring_msgs/msg/metric_data.hpp>
#include <ros_monitoring_msgs/msg/metric_dimension.hpp>
#include <ros_monitoring_msgs/msg/metric_list.hpp>
#include <std_msgs/msg/string.h>

#include <cloudwatch_metrics_collector/metrics_collector.hpp>
#include <cloudwatch_metrics_common/metric_service.hpp>
#include <cloudwatch_metrics_common/metric_service_factory.hpp>
#include <string>
#include <vector>
#include <map>
#include <std_srvs/srv/trigger.hpp>
#include <std_srvs/srv/empty.hpp>
#include <cloudwatch_metrics_collector/metrics_collector_parameter_helper.hpp>

using namespace Aws::Client;
using namespace Aws::Utils::Logging;
using namespace Aws::CloudWatchMetrics::Utils;

namespace Aws {
namespace CloudWatchMetrics {
namespace Utils {

void MetricsCollector::Initialize(std::string metric_namespace,
                         std::map<std::string, std::string> & default_dimensions,
                         int storage_resolution,
                         rclcpp::Node::SharedPtr node,
                         const Aws::Client::ClientConfiguration & config,
                         const Aws::SDKOptions & sdk_options,
                         const Aws::CloudWatchMetrics::CloudWatchOptions & cloudwatch_options,
                         std::shared_ptr<Aws::Client::ParameterReaderInterface> parameter_reader,
                         std::shared_ptr<MetricServiceFactory> metric_service_factory) {

  this->metric_namespace_ = metric_namespace;
  this->default_dimensions_ = default_dimensions;
  this->storage_resolution_.store(storage_resolution);
  this->node_ = node;
  this->metric_service_ = metric_service_factory->createMetricService(this->metric_namespace_,
                                                                      config,
                                                                      sdk_options,
                                                                      cloudwatch_options);
  this->parameter_reader_ = parameter_reader;
}

void MetricsCollector::SubscribeAllTopics()
{
  ReadTopics(parameter_reader_, topics_);
  for (auto it = topics_.begin(); it != topics_.end(); ++it) {
    auto sub = node_->create_subscription<ros_monitoring_msgs::msg::MetricList>(
            *it, kNodeSubQueueSize,
            [this](ros_monitoring_msgs::msg::MetricList::UniquePtr metric_list_msg) -> void {
                this->RecordMetrics(std::move(metric_list_msg));
            });
    subscriptions_.push_back(sub);
  }
}

int MetricsCollector::RecordMetrics(
  ros_monitoring_msgs::msg::MetricList::UniquePtr metric_list_msg)
{
  int batched_count = 0;
  AWS_LOGSTREAM_DEBUG(__func__, "Received " << metric_list_msg->metrics.size() << " metrics");

  for (auto metric_msg = metric_list_msg->metrics.begin();
       metric_msg != metric_list_msg->metrics.end(); ++metric_msg) {

    std::map<std::string, std::string> dimensions;

    for (auto it = default_dimensions_.begin(); it != default_dimensions_.end(); ++it) {
      dimensions.emplace(it->first, it->second);  // ignore the return, if we get a duplicate we're
                                                  // going to stick with the first one
    }
    for (auto it = metric_msg->dimensions.begin(); it != metric_msg->dimensions.end(); ++it) {
      dimensions.emplace((*it).name, (*it).value);  // ignore the return, if we get a duplicate
                                                    // we're going to stick with the first one
    }
    AWS_LOGSTREAM_DEBUG(__func__, "Recording metric with name=[" << metric_msg->metric_name << "]");

    // create a MetricObject with message parameters to batch
    Aws::CloudWatchMetrics::Utils::MetricObject metric_object {metric_msg->metric_name,
                                                              metric_msg->value,
                                                              metric_msg->unit,
                                                              GetMetricDataEpochMillis(*metric_msg),
                                                              dimensions,
                                                              this->storage_resolution_.load()};
    bool batched = metric_service_->batchData(metric_object);

    if (!batched) {
      AWS_LOGSTREAM_ERROR(__func__, "Failed to record metric");
    }

    batched_count++;
  }
  return batched_count;
}

int64_t MetricsCollector::GetMetricDataEpochMillis(const ros_monitoring_msgs::msg::MetricData & metric_msg)
{
  return rclcpp::Time(metric_msg.time_stamp).nanoseconds() / 1000000;
}

void MetricsCollector::TriggerPublish()
{
  AWS_LOG_DEBUG(__func__, "Flushing metrics");
  this->metric_service_->publishBatchedData();
}

bool MetricsCollector::start() {
  bool is_started = true;
  this->SubscribeAllTopics();

  if (this->metric_service_) {
    is_started &= this->metric_service_->start();
  }
  is_started &= Service::start();
  return is_started;
}

bool MetricsCollector::shutdown() {
  bool is_shutdown = Service::shutdown();
  if (this->metric_service_) {
    is_shutdown &= this->metric_service_->shutdown();
  }
  return is_shutdown;
}

bool MetricsCollector::checkIfOnline(std::shared_ptr<std_srvs::srv::Trigger::Request> , std::shared_ptr<std_srvs::srv::Trigger::Response> response) {

  AWS_LOGSTREAM_DEBUG(__func__, "received request");

  if (!this->metric_service_) {
    response->success = false;
    response->message = "The MetricsCollector is not initialized";
    return true;
  }

  response->success = this->metric_service_->isConnected();
  response->message = response->success ? "The MetricsCollector is connected" : "The MetricsCollector is not connected";

  return true;
}

}  // namespace Utils
}  // namespace CloudWatchMetrics
}  // namespace Aws


