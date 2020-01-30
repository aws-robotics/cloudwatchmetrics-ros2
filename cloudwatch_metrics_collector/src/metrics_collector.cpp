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

#include <limits>
#include <map>
#include <stdexcept>
#include <string>
#include <vector>

#include <aws/core/Aws.h>
#include <aws/core/client/ClientConfiguration.h>
#include <aws/core/utils/logging/AWSLogging.h>
#include <aws/core/utils/logging/LogMacros.h>
#include <aws/core/utils/StringUtils.h>
#include <aws/monitoring/CloudWatchClient.h>
#include <aws/monitoring/model/PutMetricDataRequest.h>
#include <aws_common/sdk_utils/client_configuration_provider.h>
#include <aws_ros2_common/sdk_utils/logging/aws_ros_logger.h>
#include <builtin_interfaces/msg/time.hpp>
#include <metrics_statistics_msgs/msg/metrics_message.hpp>
#include <metrics_statistics_msgs/msg/statistic_data_point.hpp>
#include <metrics_statistics_msgs/msg/statistic_data_type.hpp>
#include <rclcpp/rclcpp.hpp>
#include <ros_monitoring_msgs/msg/metric_data.hpp>
#include <ros_monitoring_msgs/msg/metric_dimension.hpp>
#include <ros_monitoring_msgs/msg/metric_list.hpp>
#include <std_msgs/msg/string.h>
#include <std_srvs/srv/empty.hpp>
#include <std_srvs/srv/trigger.hpp>

#include <cloudwatch_metrics_collector/metrics_collector.hpp>
#include <cloudwatch_metrics_collector/metrics_collector_parameter_helper.hpp>
#include <cloudwatch_metrics_common/metric_service.hpp>
#include <cloudwatch_metrics_common/metric_service_factory.hpp>

using namespace Aws::Client;
using namespace Aws::Utils::Logging;
using namespace Aws::CloudWatchMetrics::Utils;
using metrics_statistics_msgs::msg::StatisticDataType;

namespace Aws {
namespace CloudWatchMetrics {
namespace Utils {

void MetricsCollector::Initialize(std::string metric_namespace,
                         const std::map<std::string, std::string> & default_dimensions,
                         int storage_resolution,
                         rclcpp::Node::SharedPtr node,
                         const Aws::Client::ClientConfiguration & config,
                         const Aws::SDKOptions & sdk_options,
                         const Aws::CloudWatchMetrics::CloudWatchOptions & cloudwatch_options,
                         const std::vector<std::string> & topics,
                         std::shared_ptr<MetricServiceFactory> metric_service_factory) {

  std::vector<TopicInfo> topic_infos;
  for (const auto & topic : topics) {
    topic_infos.push_back(TopicInfo{topic, TopicType::ROS_MONITORING_MSGS});
  }
  Initialize(metric_namespace, default_dimensions, storage_resolution, node, config,
    sdk_options, cloudwatch_options, topic_infos, metric_service_factory);
}

void MetricsCollector::Initialize(std::string metric_namespace,
                         const std::map<std::string, std::string> & default_dimensions,
                         int storage_resolution,
                         rclcpp::Node::SharedPtr node,
                         const Aws::Client::ClientConfiguration & config,
                         const Aws::SDKOptions & sdk_options,
                         const Aws::CloudWatchMetrics::CloudWatchOptions & cloudwatch_options,
                         const std::vector<TopicInfo> & topics,
                         std::shared_ptr<MetricServiceFactory> metric_service_factory) {

  this->metric_namespace_ = metric_namespace;
  this->default_dimensions_ = default_dimensions;
  this->storage_resolution_.store(storage_resolution);
  this->node_ = node;
  this->metric_service_ = metric_service_factory->createMetricService(this->metric_namespace_,
                                                                      config,
                                                                      sdk_options,
                                                                      cloudwatch_options);
  this->topics_ = topics;
}

void MetricsCollector::SubscribeAllTopics()
{
  for (const auto & topic_info : topics_) {
    std::shared_ptr<rclcpp::SubscriptionBase> sub;
    if (topic_info.topic_type == TopicType::ROS_MONITORING_MSGS) {
      sub = node_->create_subscription<ros_monitoring_msgs::msg::MetricList>(
              topic_info.topic_name, kNodeSubQueueSize,
              [this](ros_monitoring_msgs::msg::MetricList::UniquePtr metric_list_msg) -> void {
                this->RecordMetrics(std::move(metric_list_msg));
              });
    } else if (topic_info.topic_type == TopicType::METRICS_STATISTICS_MSGS) {
      sub = node_->create_subscription<metrics_statistics_msgs::msg::MetricsMessage>(
              topic_info.topic_name, kNodeSubQueueSize,
              [this](metrics_statistics_msgs::msg::MetricsMessage::UniquePtr msg) -> void {
                this->RecordMetrics(std::move(msg));
              });
    }
    subscriptions_.push_back(std::move(sub));
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
    Aws::CloudWatchMetrics::Utils::MetricObject metric_object{metric_msg->metric_name,
                                                              metric_msg->value,
                                                              metric_msg->unit,
                                                              GetMetricDataEpochMillis(metric_msg->time_stamp),
                                                              dimensions,
                                                              this->storage_resolution_.load()};
    bool batched = metric_service_->batchData(metric_object);

    if (!batched) {
      AWS_LOGSTREAM_ERROR(__func__, "Failed to record ROS monitoring message");
    } else {
      ++batched_count;
    }
  }
  return batched_count;
}

int MetricsCollector::RecordMetrics(
  metrics_statistics_msgs::msg::MetricsMessage::UniquePtr msg)
{
  int batched_count = 0;

  AWS_LOGSTREAM_DEBUG(__func__, "Received metric with " << msg->statistics.size() << " types of statistics");
  AWS_LOGSTREAM_DEBUG(__func__, "Recording metric with name=[" << msg->measurement_source_name << "]");

  std::map<std::string, std::string> dimensions;
  dimensions["Measurement Source"] = msg->measurement_source_name;

  std::map<StatisticValuesType, double> statistic_values;
  double stddev;
  for (const auto & statistic : msg->statistics) {
    if (statistic.data_type == StatisticDataType::STATISTICS_DATA_TYPE_AVERAGE) {
      statistic_values[StatisticValuesType::SUM] = statistic.data;
    } else if (statistic.data_type == StatisticDataType::STATISTICS_DATA_TYPE_MAXIMUM) {
      statistic_values[StatisticValuesType::MAXIMUM] = statistic.data;
    } else if (statistic.data_type == StatisticDataType::STATISTICS_DATA_TYPE_MINIMUM) {
      statistic_values[StatisticValuesType::MINIMUM] = statistic.data;
    } else if (statistic.data_type == StatisticDataType::STATISTICS_DATA_TYPE_STDDEV) {
      stddev = statistic.data;
    } else if (statistic.data_type == StatisticDataType::STATISTICS_DATA_TYPE_SAMPLE_COUNT) {
      statistic_values[StatisticValuesType::SAMPLE_COUNT] = statistic.data;
    }
  }
  try {
    double & sum = statistic_values.at(StatisticValuesType::SUM);
    sum *= statistic_values.at(StatisticValuesType::SAMPLE_COUNT);
  } catch (const std::out_of_range & exception) {
    statistic_values.erase(StatisticValuesType::SUM);
  }

  // create a MetricObject with message parameters to batch
  MetricObject metric_object(
    msg->metrics_source,
    statistic_values,
    msg->unit,
    GetMetricDataEpochMillis(msg->window_start),
    dimensions,
    storage_resolution_.load()
  );

  bool batched = metric_service_->batchData(metric_object);
  if (!batched) {
    AWS_LOGSTREAM_ERROR(__func__, "Failed to record metrics statistics message");
  } else {
    ++batched_count;
  }

  MetricObject stddev_object(
    msg->metrics_source + "_stddev",
    stddev,
    msg->unit,
    GetMetricDataEpochMillis(msg->window_start),
    dimensions,
    storage_resolution_.load()
  );

  batched = metric_service_->batchData(stddev_object);
  if (!batched) {
    AWS_LOGSTREAM_ERROR(__func__, "Failed to record metrics stddev message");
  } else {
    ++batched_count;
  }

  return batched_count;
}

int64_t MetricsCollector::GetMetricDataEpochMillis(const builtin_interfaces::msg::Time & time_stamp)
{
  return rclcpp::Time(time_stamp).nanoseconds() / 1000000;
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
