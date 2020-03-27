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

#include <limits>
#include <map>
#include <memory>
#include <random>
#include <vector>

#include <ros_monitoring_msgs/msg/metric_data.hpp>

#include <cloudwatch_metrics_common/metric_service.hpp>
#include <cloudwatch_metrics_common/metric_service_factory.hpp>
#include <cloudwatch_metrics_collector/metrics_collector.hpp>

#include "test_common.hpp"

#include <gtest/gtest.h>

using namespace Aws::CloudWatchMetrics::Utils;
using namespace Aws::CloudWatchMetrics;

class MetricServiceMock : public MetricService
{
public:
  explicit MetricServiceMock(const std::string & metrics_namespace) : MetricService(
    std::make_shared<MetricPublisher>(metrics_namespace, Aws::Client::ClientConfiguration()),
    std::make_shared<MetricBatcher>()) {}

  bool batchData(const MetricObject & data_to_batch) override
  {
    metric_objects_.push_back(data_to_batch);
    return true;
  }

  const std::vector<MetricObject> & GetMetricObjects()
  {
    return metric_objects_;
  }

private:
  std::vector<MetricObject> metric_objects_;
};

class MetricServiceFactoryMock : public MetricServiceFactory
{
public:
  explicit MetricServiceFactoryMock(std::shared_ptr<MetricService> metric_service)
    : mock_metric_service_(std::move(metric_service)) {}

  std::shared_ptr<MetricService> createMetricService(
    const std::string & /*metrics_namespace*/,
    const Aws::Client::ClientConfiguration & /*client_config*/,
    const Aws::SDKOptions & /*sdk_options*/,
    const CloudWatchOptions & /*cloudwatch_option*/) override
  {
    return mock_metric_service_;
  }

private:
  std::shared_ptr<MetricService> mock_metric_service_;
};

class MetricsCollectorFixture : public ::testing::Test
{
  void SetUp() override
  {
    mock_metric_service_ = std::make_shared<MetricServiceMock>(kTestMetricsNamespace);
    mock_metric_service_factory_ = std::make_shared<MetricServiceFactoryMock>(mock_metric_service_);

    metrics_collector_.Initialize(
      kTestMetricsNamespace,
      default_dimensions_,
      60,
      nullptr,
      client_config_,
      sdk_options_,
      cloudwatch_options_,
      topics_,
      mock_metric_service_factory_);
  }

protected:
  static constexpr char kTestMetricsNamespace[] = "test_namespace";

  Aws::Client::ClientConfiguration client_config_;
  Aws::SDKOptions sdk_options_;
  Aws::CloudWatchMetrics::CloudWatchOptions cloudwatch_options_;
  std::vector<std::string> topics_;

  std::map<std::string, std::string> default_dimensions_;
  std::shared_ptr<MetricServiceMock> mock_metric_service_;
  std::shared_ptr<MetricServiceFactoryMock> mock_metric_service_factory_;
  MetricsCollector metrics_collector_;
};

constexpr char MetricsCollectorFixture::kTestMetricsNamespace[];

TEST_F(MetricsCollectorFixture, RecordRosMonitoringMessage)
{
  ros_monitoring_msgs::msg::MetricData metric_data = EmptyMonitoringData();
  metric_data.value = 9001.0;

  auto metric_list = std::make_unique<ros_monitoring_msgs::msg::MetricList>();
  metric_list->metrics.push_back(metric_data);

  int num_batched = metrics_collector_.RecordMetrics(std::move(metric_list));
  ASSERT_EQ(1, num_batched);

  const auto & metric_objs = mock_metric_service_->GetMetricObjects();
  ASSERT_EQ(static_cast<size_t>(num_batched), metric_objs.size());

  EXPECT_EQ(metric_data.metric_name, metric_objs[0].metric_name);
  EXPECT_EQ(metric_data.unit, metric_objs[0].unit);
  EXPECT_DOUBLE_EQ(metric_data.value, metric_objs[0].value);
  EXPECT_EQ(MetricsCollector::GetMetricDataEpochMillis(metric_data.time_stamp), metric_objs[0].timestamp);

  std::map<std::string, std::string> dimensions;
  for (const auto & dimension : metric_data.dimensions) {
    dimensions[dimension.name] = dimension.value;
  }
  EXPECT_EQ(dimensions, metric_objs[0].dimensions);
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
