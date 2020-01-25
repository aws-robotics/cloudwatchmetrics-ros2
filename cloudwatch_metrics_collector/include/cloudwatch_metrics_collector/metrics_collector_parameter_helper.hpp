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

#include <array>
#include <iostream>
#include <string>
#include <unordered_set>

#include <rclcpp/rclcpp.hpp>

#include <aws_common/sdk_utils/aws_error.h>
#include <aws_common/sdk_utils/parameter_reader.h>
#include <cloudwatch_metrics_common/cloudwatch_options.h>

using namespace Aws::Client;

namespace Aws {
namespace CloudWatchMetrics {
namespace Utils {

constexpr char kNodeParamMonitorTopicsListKey[] = "aws_monitored_metric_topics";
constexpr char kNodeParamMonitorTopicTypesListKey[] = "aws_monitored_metric_topic_types";
constexpr char kNodeParamMetricNamespaceKey[] = "aws_metrics_namespace";
constexpr char kNodeParamDefaultMetricDimensionsKey[]  = "aws_default_metric_dimensions";
constexpr char kNodeName[] = "cloudwatch_metrics_collector";
constexpr char kNodeParamPublishFrequencyKey[] = "publish_frequency";

/** Configuration params for Aws::DataFlow::UploaderOptions **/
constexpr char kNodeParamBatchMaxQueueSize[] = "batch_max_queue_size";
constexpr char kNodeParamBatchTriggerPublishSize[] = "batch_trigger_publish_size";
constexpr char kNodeParamFileMaxQueueSize[] = "file_max_queue_size";
constexpr char kNodeParamFileUploadBatchSize[] = "file_upload_batch_size";
constexpr char kNodeParamStreamMaxQueueSize[] = "stream_max_queue_size";

/** Configuration params for Aws::FileManagement::FileManagerStrategyOptions **/
constexpr char kNodeParamFileExtension[] = "file_extension";
constexpr char kNodeParamFilePrefix[] = "file_prefix";
constexpr char kNodeParamMaximumFileSize[] = "maximum_file_size";
constexpr char kNodeParamStorageDirectory[] = "storage_directory";
constexpr char kNodeParamStorageLimit[] = "storage_limit";

enum class TopicType
{
  ROS_MONITORING_MSGS = 1,
  METRICS_STATISTICS_MSGS = 2
};

struct TopicInfo
{
  std::string topic_name;
  TopicType topic_type;
};

constexpr int kNodeSubQueueSize = 100;
constexpr int kNodePublishFrequencyDefaultValue = 10;
constexpr int kNodeMetricServiceTimeSec = 1;
constexpr char kNodeDefaultMetricNamespace[] = "ROS";
const std::array<TopicInfo, 2> kNodeDefaultMetricsTopics = {{
  {"metrics", TopicType::ROS_MONITORING_MSGS},
  {"system_metrics", TopicType::METRICS_STATISTICS_MSGS}
}};
constexpr int kNodeDefaultMetricDatumStorageResolution = 60;
constexpr char kNodeParamMetricDatumStorageResolutionKey[] = "storage_resolution";
const std::set<int> kNodeParamMetricDatumStorageResolutionValidValues = {1, 60};


/**
 * Fetch the parameter for the log publishing frequency.
 *
 * @param parameter_reader to retrieve the parameters from.
 * @param publish_frequency the parameter is stored here when it is read successfully.
 * @return an error code that indicates whether the parameter was read successfully or not,
 * as returned by \p parameter_reader
 */
void ReadPublishFrequency(
        std::shared_ptr<Aws::Client::ParameterReaderInterface> parameter_reader,
        int & publish_frequency);

/**
 *
 * @param parameter_reader
 * @param metric_namespace
 * @return
 */
void ReadMetricNamespace(
        std::shared_ptr<Aws::Client::ParameterReaderInterface> parameter_reader,
        std::string & metric_namespace);

/**
 *
 * @param parameter_reader
 * @param dimensions_param
 * @param metric_dims
 * @return
 */
void ReadMetricDimensions(
        std::shared_ptr<Aws::Client::ParameterReaderInterface> parameter_reader,
        Aws::String & dimensions_param,
        std::map<std::string, std::string> & metric_dims);
/**
 *
 * @param parameter_reader
 * @param storage_resolution
 * @return
 */
void ReadStorageResolution(
  std::shared_ptr<Aws::Client::ParameterReaderInterface> parameter_reader,
  int & storage_resolution);

/**
 *
 * @param parameter_reader
 * @param topics 
 * @return
 */
void ReadTopics(
  std::shared_ptr<Aws::Client::ParameterReaderInterface> parameter_reader,
  std::vector<TopicInfo> & topics);

/**
 * Fetch the options related to cloudwatch uploading and offline file mangement
 *
 * @param parameter_reader to retrieve the parameters from
 * @param cloudwatch_options a struct of uploader and file_manager options
 * @return an error code that indicates whether the parameter was read successfully or not,
 * as returned by \p parameter_reader
 */
void ReadCloudWatchOptions(
  std::shared_ptr<Aws::Client::ParameterReaderInterface> parameter_reader,
  Aws::CloudWatchMetrics::CloudWatchOptions & cloudwatch_options);

/**
 * Fetch the options related to cloudwatch log uploading
 *
 * @param parameter_reader to retrieve the parameters from
 * @param uploader_options a struct of uploader options
 * @return an error code that indicates whether the parameter was read successfully or not,
 * as returned by \p parameter_reader
 */
void ReadUploaderOptions(
  std::shared_ptr<Aws::Client::ParameterReaderInterface> parameter_reader,
  Aws::DataFlow::UploaderOptions & uploader_options);

/**
 * Fetch the options related to cloudwatch offline file management
 *
 * @param parameter_reader to retrieve the parameters from
 * @param file_manager_strategy_options a struct of file management options
 * @return an error code that indicates whether the parameter was read successfully or not,
 * as returned by \p parameter_reader
 */
void ReadFileManagerStrategyOptions(
  std::shared_ptr<Aws::Client::ParameterReaderInterface> parameter_reader,
  Aws::FileManagement::FileManagerStrategyOptions & file_manager_strategy_options);

/**
 * Fetch a single string option
 *
 * @param parameter_reader to retrieve the parameters from
 * @param option_key the parameter key to read
 * @param default_value a default value if the parameter doesn't exist or is unreadble
 * @param option_value the string value for this option
 * @return an error code that indicates whether the parameter was read successfully or not,
 * as returned by \p parameter_reader
 */
void ReadOption(
  std::shared_ptr<Aws::Client::ParameterReaderInterface> parameter_reader,
  const std::string & option_key,
  const std::string & default_value,
  std::string & option_value);

/**
 * Fetch a single size_t option
 *
 * @param parameter_reader to retrieve the parameters from
 * @param option_key the parameter key to read
 * @param default_value a default value if the parameter doesn't exist or is unreadble
 * @param option_value the size_t value for this option
 * @return an error code that indicates whether the parameter was read successfully or not,
 * as returned by \p parameter_reader
 */
void ReadOption(
  std::shared_ptr<Aws::Client::ParameterReaderInterface> parameter_reader,
  const std::string & option_key,
  const size_t & default_value,
  size_t & option_value);

}  // namespace Utils
}  // namespace CloudWatchMetrics
}  // namespace Aws
