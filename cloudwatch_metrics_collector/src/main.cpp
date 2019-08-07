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
#include <ros_monitoring_msgs/msg/metric_data.h>
#include <ros_monitoring_msgs/msg/metric_dimension.h>
#include <ros_monitoring_msgs/msg/metric_list.h>
#include <std_msgs/msg/string.h>

#include <cloudwatch_metrics_collector/metrics_collector.hpp>
#include <cloudwatch_metrics_common/metric_service.hpp>
#include <cloudwatch_metrics_common/metric_service_factory.hpp>
#include <chrono>
#include <string>
#include <vector>
#include <map>

#include <aws_common/sdk_utils/parameter_reader.h>

#include <cloudwatch_metrics_collector/metrics_collector_parameter_helper.hpp>
#include <cloudwatch_metrics_common/cloudwatch_options.h>

using namespace Aws::CloudWatchMetrics::Utils;

int main(int argc, char * argv[])
{
  int status = 0;
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions node_options;
  node_options.allow_undeclared_parameters(true);
  node_options.automatically_declare_parameters_from_overrides(true);
  auto node = rclcpp::Node::make_shared(kNodeName, node_options);

  std::vector<std::shared_ptr<rclcpp::Subscription<ros_monitoring_msgs::msg::MetricData>>> subscriptions;

  // initialize SDK logging
  Aws::Utils::Logging::InitializeAWSLogging(Aws::MakeShared<Aws::Utils::Logging::AWSROSLogger>(kNodeName.c_str(),
    Aws::Utils::Logging::LogLevel::Trace, node));

  //-----------Start Read Configuration Parameters---------------------

  std::shared_ptr<Aws::Client::ParameterReaderInterface> parameter_reader =
          std::make_shared<Aws::Client::Ros2NodeParameterReader>(node);

  //SDK client config
  ClientConfigurationProvider client_config_provider(parameter_reader);
  ClientConfiguration client_config = client_config_provider.GetClientConfiguration();

  Aws::SDKOptions sdk_options;

  int publish_frequency;

  std::string metric_namespace;
  Aws::String dimensions_param;
  std::map<std::string, std::string> default_metric_dims;
  std::vector<std::string> topics;

  // Load the storage resolution
  int storage_resolution = kNodeDefaultMetricDatumStorageResolution;
  Aws::CloudWatchMetrics::CloudWatchOptions cloudwatch_options;

  ReadPublishFrequency(parameter_reader, publish_frequency);
  ReadMetricNamespace(parameter_reader, metric_namespace);
  ReadMetricDimensions(parameter_reader, dimensions_param, default_metric_dims);
  ReadStorageResolution(parameter_reader, storage_resolution);

  ReadCloudWatchOptions(parameter_reader, cloudwatch_options);
  ReadTopics(parameter_reader, topics);
  //-----------------End read configuration parameters-----------------------

  // create the metric collector
  Aws::CloudWatchMetrics::Utils::MetricsCollector metrics_collector;

  // initialize with options read from the config file
  metrics_collector.Initialize(
          metric_namespace,
          default_metric_dims,
          storage_resolution,
          node,
          client_config,
          sdk_options,
          cloudwatch_options,
          topics);

  auto is_online_callback =
    [&metrics_collector](const std::shared_ptr<rmw_request_id_t> request_header,
        const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
        std::shared_ptr<std_srvs::srv::Trigger::Response> response) -> void
    {
      (void) request_header;
      metrics_collector.checkIfOnline(request, response);
    };
      
  auto service = node->create_service<std_srvs::srv::Trigger>(kNodeName,
                                                              is_online_callback);

  // start the collection process
  metrics_collector.start();

  bool publish_when_size_reached = cloudwatch_options.uploader_options.batch_trigger_publish_size
    != Aws::DataFlow::kDefaultUploaderOptions.batch_trigger_publish_size;

  rclcpp::TimerBase::SharedPtr timer;
  // Publish on a timer if we are not publishing on a size limit.
  if (!publish_when_size_reached) {
    timer =
      node->create_wall_timer(std::chrono::seconds(publish_frequency),
                              std::bind(&Aws::CloudWatchMetrics::Utils::MetricsCollector::TriggerPublish, &metrics_collector));
  }

  rclcpp::spin(node);

  AWS_LOG_INFO(__func__, "Shutting down Metrics Collector ...");
  metrics_collector.shutdown();
  Aws::Utils::Logging::ShutdownAWSLogging();
  rclcpp::shutdown();
  return status;
}
