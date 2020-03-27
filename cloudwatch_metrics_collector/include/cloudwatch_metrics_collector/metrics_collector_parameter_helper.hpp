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

namespace Aws {
namespace CloudWatchMetrics {
namespace Utils {

constexpr char kNodeParamMonitoringTopicsListKey[] = "aws_monitored_metric_topics";
constexpr char kNodeParamMetricNamespaceKey[] = "aws_metrics_namespace";
constexpr char kNodeParamDefaultMetricDimensionsKey[]  = "aws_default_metric_dimensions";
constexpr char kNodeName[] = "cloudwatch_metrics_collector";
constexpr char kNodeParamPublishFrequencyKey[] = "publish_frequency";
constexpr char kNodeParamMetricDatumStorageResolutionKey[] = "storage_resolution";

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

constexpr int kNodeSubQueueSize = 100;
constexpr int kNodePublishFrequencyDefaultValue = 10;
constexpr int kNodeMetricServiceTimeSec = 1;
constexpr char kNodeDefaultMetricNamespace[] = "ROS";
constexpr char kNodeDefaultMetricsTopic[] = "metrics";
constexpr int kNodeDefaultMetricDatumStorageResolution = 60;
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
        const std::shared_ptr<Aws::Client::ParameterReaderInterface>& parameter_reader,
        int & publish_frequency);

/**
 *
 * @param parameter_reader
 * @param metric_namespace
 * @return
 */
void ReadMetricNamespace(
        const std::shared_ptr<Aws::Client::ParameterReaderInterface>& parameter_reader,
        std::string & metric_namespace);

/**
 *
 * @param parameter_reader
 * @param dimensions_param
 * @param metric_dims
 * @return
 */
void ReadMetricDimensions(
        const std::shared_ptr<Aws::Client::ParameterReaderInterface>& parameter_reader,
        Aws::String & dimensions_param,
        std::map<std::string, std::string> & metric_dims);
/**
 *
 * @param parameter_reader
 * @param storage_resolution
 * @return
 */
void ReadStorageResolution(
  const std::shared_ptr<Aws::Client::ParameterReaderInterface>& parameter_reader,
  int & storage_resolution);

/**
 *
 * @param parameter_reader
 * @param topics 
 * @return
 */
void ReadTopics(
  const std::shared_ptr<Aws::Client::ParameterReaderInterface>& parameter_reader,
  std::vector<std::string> & topics);

/**
 * Fetch the options related to cloudwatch uploading and offline file mangement
 *
 * @param parameter_reader to retrieve the parameters from
 * @param cloudwatch_options a struct of uploader and file_manager options
 * @return an error code that indicates whether the parameter was read successfully or not,
 * as returned by \p parameter_reader
 */
void ReadCloudWatchOptions(
  const std::shared_ptr<Aws::Client::ParameterReaderInterface>& parameter_reader,
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
  const std::shared_ptr<Aws::Client::ParameterReaderInterface>& parameter_reader,
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
  const std::shared_ptr<Aws::Client::ParameterReaderInterface>& parameter_reader,
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
  const std::shared_ptr<Aws::Client::ParameterReaderInterface>& parameter_reader,
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
  const std::shared_ptr<Aws::Client::ParameterReaderInterface>& parameter_reader,
  const std::string & option_key,
  const size_t & default_value,
  size_t & option_value);

}  // namespace Utils
}  // namespace CloudWatchMetrics
}  // namespace Aws
