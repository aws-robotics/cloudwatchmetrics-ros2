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

#include <metrics_statistics_msgs/msg/metrics_message.hpp>
#include <ros_monitoring_msgs/msg/metric_data.hpp>

#include <cloudwatch_metrics_common/metric_service.hpp>
#include <cloudwatch_metrics_common/metric_service_factory.hpp>
#include <cloudwatch_metrics_collector/metrics_collector.hpp>

#include "test_common.hpp"

#include <gtest/gtest.h>

using namespace Aws::CloudWatchMetrics::Utils;
using namespace Aws::CloudWatchMetrics;
using metrics_statistics_msgs::msg::StatisticDataType;

class MetricServiceMock : public MetricService
{
public:
  MetricServiceMock(const std::string & metrics_namespace) : MetricService(
    std::make_shared<MetricPublisher>(metrics_namespace, Aws::Client::ClientConfiguration()),
    std::make_shared<MetricBatcher>()) {}

  bool batchData(const MetricObject & data_to_batch)
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
  MetricServiceFactoryMock(std::shared_ptr<MetricService> metric_service)
    : mock_metric_service_(std::move(metric_service)) {}

  std::shared_ptr<MetricService> createMetricService(
    const std::string & /*metrics_namespace*/,
    const Aws::Client::ClientConfiguration & /*client_config*/,
    const Aws::SDKOptions & /*sdk_options*/,
    const CloudWatchOptions & /*cloudwatch_option*/)
  {
    return mock_metric_service_;
  }

private:
  std::shared_ptr<MetricService> mock_metric_service_;
};

class MetricsCollectorFixture : public ::testing::Test
{
  void SetUp()
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
  std::vector<TopicInfo> topics_;

  std::map<std::string, std::string> default_dimensions_;
  std::shared_ptr<MetricServiceMock> mock_metric_service_;
  std::shared_ptr<MetricServiceFactoryMock> mock_metric_service_factory_;
  MetricsCollector metrics_collector_;
};

constexpr char MetricsCollectorFixture::kTestMetricsNamespace[];

TEST_F(MetricsCollectorFixture, RecordRosMonitoringMessages)
{
  std::random_device rd;
  std::mt19937 gen(rd());
  std::uniform_real_distribution<> dist(0.0, 100.0);

  std::array<ros_monitoring_msgs::msg::MetricData, 2> metric_data = {{
    BasicMonitoringData(),
    BasicMonitoringData()
  }};
  for (auto & data : metric_data) {
    data.value = dist(gen);
  }

  auto metric_list = std::make_unique<ros_monitoring_msgs::msg::MetricList>();
  for (const auto & data : metric_data) {
    metric_list->metrics.push_back(data);
  }

  size_t num_batched = metrics_collector_.RecordMetrics(std::move(metric_list));
  ASSERT_EQ(metric_data.size(), num_batched);

  const auto & metric_objs = mock_metric_service_->GetMetricObjects();
  ASSERT_EQ(metric_data.size(), metric_objs.size());

  for (size_t i = 0; i < metric_data.size(); ++i) {
    EXPECT_EQ(metric_data[i].metric_name, metric_objs[i].metric_name);
    EXPECT_EQ(metric_data[i].unit, metric_objs[i].unit);
    if (std::isnan(metric_data[i].value)) {
      EXPECT_TRUE(std::isnan(metric_objs[i].value));
    } else {
      EXPECT_DOUBLE_EQ(metric_data[i].value, metric_objs[i].value);
    }
    EXPECT_EQ(MetricsCollector::GetMetricDataEpochMillis(metric_data[i].time_stamp), metric_objs[i].timestamp);

    std::map<std::string, std::string> dimensions;
    for (const auto & dimension : metric_data[i].dimensions) {
      dimensions[dimension.name] = dimension.value;
    }
    EXPECT_EQ(dimensions, metric_objs[i].dimensions);
  }
}

TEST_F(MetricsCollectorFixture, RecordMetricsMessages)
{
  std::random_device rd;
  std::mt19937 gen(rd());
  std::uniform_real_distribution<> dist(0.0, 100.0);

  std::array<metrics_statistics_msgs::msg::MetricsMessage, 2> metric_data = {{
    BasicMetricsData(),
    BasicMetricsData()
  }};
  for (auto & metric : metric_data) {
    for (auto & statistic : metric.statistics) {
      statistic.data = dist(gen);
    }
  }

  for (const auto & data : metric_data) {
    auto msg = std::make_unique<metrics_statistics_msgs::msg::MetricsMessage>(data);
    int num_batched = metrics_collector_.RecordMetrics(std::move(msg));
    ASSERT_EQ(2, num_batched);
  }

  const auto & metric_objs = mock_metric_service_->GetMetricObjects();
  ASSERT_EQ(2 * metric_data.size(), metric_objs.size());

  for (size_t i = 0; i < metric_data.size(); ++i) {
    EXPECT_EQ(metric_data[i].measurement_source_name, metric_objs[2*i].metric_name);
    EXPECT_EQ(metric_data[i].measurement_source_name+"_stddev", metric_objs[2*i+1].metric_name);

    EXPECT_EQ(metric_data[i].metrics_source, metric_objs[2*i].dimensions.at("metric_type"));
    EXPECT_EQ(metric_data[i].metrics_source, metric_objs[2*i+1].dimensions.at("metric_type"));

    EXPECT_EQ(metric_data[i].unit, metric_objs[2*i].dimensions.at("metric_unit"));
    EXPECT_EQ(metric_data[i].unit, metric_objs[2*i+1].dimensions.at("metric_unit"));

    EXPECT_EQ(MetricsCollector::GetMetricDataEpochMillis(metric_data[i].window_start), metric_objs[2*i].timestamp);
    EXPECT_EQ(MetricsCollector::GetMetricDataEpochMillis(metric_data[i].window_start), metric_objs[2*i+1].timestamp);

    EXPECT_EQ(MetricsCollector::GetMetricDataEpochMillis(metric_data[i].window_stop)
      - MetricsCollector::GetMetricDataEpochMillis(metric_data[i].window_start), metric_objs[2*i].value);

    for (const auto & statistic : metric_data[i].statistics) {
      if (statistic.data_type == StatisticDataType::STATISTICS_DATA_TYPE_AVERAGE) {
        if (std::isnan(statistic.data)) {
          EXPECT_TRUE(std::isnan(metric_objs[2*i].statistic_values.at(StatisticValuesType::SUM))
            || std::isnan(metric_objs[2*i].statistic_values.at(StatisticValuesType::SAMPLE_COUNT)));
        } else {
          EXPECT_DOUBLE_EQ(statistic.data,
            metric_objs[2*i].statistic_values.at(StatisticValuesType::SUM)
            / metric_objs[2*i].statistic_values.at(StatisticValuesType::SAMPLE_COUNT));
        }
      } else if (statistic.data_type == StatisticDataType::STATISTICS_DATA_TYPE_MAXIMUM) {
        if (std::isnan(statistic.data)) {
          EXPECT_TRUE(std::isnan(metric_objs[2*i].statistic_values.at(StatisticValuesType::MAXIMUM)));
        } else {
          EXPECT_DOUBLE_EQ(statistic.data, metric_objs[2*i].statistic_values.at(StatisticValuesType::MAXIMUM));
        }
      } else if (statistic.data_type == StatisticDataType::STATISTICS_DATA_TYPE_MINIMUM) {
        if (std::isnan(statistic.data)) {
          EXPECT_TRUE(std::isnan(metric_objs[2*i].statistic_values.at(StatisticValuesType::MINIMUM)));
        } else {
          EXPECT_DOUBLE_EQ(statistic.data, metric_objs[2*i].statistic_values.at(StatisticValuesType::MINIMUM));
        }
      } else if (statistic.data_type == StatisticDataType::STATISTICS_DATA_TYPE_SAMPLE_COUNT) {
        if (std::isnan(statistic.data)) {
          EXPECT_TRUE(std::isnan(metric_objs[2*i].statistic_values.at(StatisticValuesType::SAMPLE_COUNT)));
        } else {
          EXPECT_DOUBLE_EQ(statistic.data, metric_objs[2*i].statistic_values.at(StatisticValuesType::SAMPLE_COUNT));
        }
      } else if (statistic.data_type == StatisticDataType::STATISTICS_DATA_TYPE_STDDEV) {
        if (std::isnan(statistic.data)) {
          EXPECT_TRUE(std::isnan(metric_objs[2*i+1].value));
        } else {
          EXPECT_DOUBLE_EQ(statistic.data, metric_objs[2*i+1].value);
        }
      }
    }
    EXPECT_TRUE(metric_objs[2*i+1].statistic_values.empty());
  }
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
