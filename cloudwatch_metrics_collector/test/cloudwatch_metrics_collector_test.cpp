/*
 * Copyright 2018 Amazon.com, Inc. or its affiliates. All Rights Reserved.
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

#include <chrono>

#include <aws/core/Aws.h>
#include <aws/core/utils/logging/LogMacros.h>

#include <rclcpp/rclcpp.hpp>

#include <cloudwatch_metrics_common/metric_service.hpp>
#include <cloudwatch_metrics_common/metric_batcher.h>
#include <cloudwatch_metrics_common/metric_publisher.hpp>
#include <cloudwatch_metrics_common/metric_service_factory.hpp>
#include <cloudwatch_metrics_collector/metrics_collector.hpp>
#include <cloudwatch_metrics_collector/metrics_collector_parameter_helper.hpp>

#include <ros_monitoring_msgs/msg/metric_data.hpp>

#include <gmock/gmock.h>
#include <gtest/gtest.h>


using namespace Aws::CloudWatchMetrics;
using namespace Aws::CloudWatchMetrics::Utils;
using ::testing::_;
using ::testing::AllOf;
using ::testing::HasSubstr;
using ::testing::Return;
using ::testing::StrEq;
using ::testing::Eq;
using ::testing::InSequence;

int test_argc;
char ** test_argv;

class MetricServiceFactoryMock : public MetricServiceFactory
{
public:
  MOCK_METHOD4(createMetricService,
               std::shared_ptr<MetricService>(
                 const std::string & metrics_namespace,
                 const Aws::Client::ClientConfiguration & client_config,
                 const Aws::SDKOptions & sdk_options,
                 const CloudWatchOptions & cloudwatch_option)
              );
};

class MetricBatcherMock : public MetricBatcher
{
public:

  MOCK_METHOD0(publishBatchedData, bool());
};

class MetricPublisherMock : public MetricPublisher
{
public:
  MetricPublisherMock(const std::string & metrics_namespace,
                      const Aws::Client::ClientConfiguration & client_config)
    : MetricPublisher(metrics_namespace, client_config) {}
};

class MetricServiceMock : public MetricService
{
public:
  MetricServiceMock(std::shared_ptr<Publisher<MetricDatumCollection>> publisher,
                    std::shared_ptr<DataBatcher<MetricDatum>> batcher,
                    std::shared_ptr<FileUploadStreamer<MetricDatumCollection>> file_upload_streamer = nullptr)
    : MetricService(publisher, batcher, file_upload_streamer) {}

  MOCK_METHOD1(batchData, bool(const MetricObject & data_to_batch));
  MOCK_METHOD0(start, bool());
  MOCK_METHOD0(shutdown, bool());
  MOCK_METHOD0(publishBatchedData, bool());
};


class MetricsCollectorFixture : public ::testing::Test
{
protected:
  const std::string kMetricsTopic = "metrics";
  const std::string kMetricName1 = "CWMetricsNodeTestMetric";
  const std::string kMetricUnit1 = "sec";
  const std::string metric_namespace = "test_namespace";

  Aws::Client::ClientConfiguration config;
  Aws::SDKOptions sdk_options;
  Aws::CloudWatchMetrics::CloudWatchOptions cloudwatch_options;

  std::shared_ptr<MetricServiceMock> metric_service;
  std::shared_ptr<MetricPublisherMock> metric_publisher;
  std::shared_ptr<MetricBatcherMock> metric_batcher;
  std::shared_ptr<MetricsCollector> metrics_collector;

  rclcpp::Node::SharedPtr node_handle;
  rclcpp::Publisher<ros_monitoring_msgs::msg::MetricList>::SharedPtr metrics_pub;

  void SetUp()
  {
    metric_batcher = std::make_shared<MetricBatcherMock>();
    metric_publisher = std::make_shared<MetricPublisherMock>(metric_namespace, config);
    metric_service = std::make_shared<MetricServiceMock>(metric_publisher, metric_batcher);
  }

  void Initialize(std::map<std::string, std::string> metric_dimensions,
                  const std::vector<std::string> & topics)
  {

    rclcpp::init(test_argc, test_argv);
    metrics_collector = std::make_shared<MetricsCollector>();

    rclcpp::NodeOptions node_options;
    node_options.allow_undeclared_parameters(true);
    node_options.automatically_declare_parameters_from_overrides(true);
    node_handle = std::make_shared<rclcpp::Node>("CWMetricsNodeTest", node_options);

    metrics_pub = node_handle->create_publisher<ros_monitoring_msgs::msg::MetricList>(kMetricsTopic.c_str(), 1);

    EXPECT_CALL(*metric_service, start()).Times(1);

    std::shared_ptr<MetricServiceFactoryMock> metric_factory_mock = std::make_shared<MetricServiceFactoryMock>();

    EXPECT_CALL(*metric_factory_mock,
                createMetricService(StrEq(metric_namespace), _, _, _))
    .WillOnce(Return(metric_service));

    metrics_collector->Initialize(metric_namespace,
                                  metric_dimensions,
                                  60,
                                  node_handle,
                                  config,
                                  sdk_options,
                                  cloudwatch_options,
                                  topics,
                                  metric_factory_mock);

    metrics_collector->start();
  }

  void TearDown()
  {
    if (metrics_collector) {
      EXPECT_CALL(*metric_service, shutdown()).Times(1);
      metrics_collector->shutdown();
    }
    rclcpp::shutdown();
  }

  void SendMetricMessages(int num_msgs, ros_monitoring_msgs::msg::MetricData & metric_data_proto)
  {
    ros_monitoring_msgs::msg::MetricList metric_list_msg = ros_monitoring_msgs::msg::MetricList();
    for (int i = 0; i < num_msgs; i++) {
      metric_data_proto.value = i;
      metric_data_proto.time_stamp = node_handle->now();
      metric_list_msg.metrics.clear();
      metric_list_msg.metrics.push_back(metric_data_proto);
      AWS_LOGSTREAM_DEBUG(__func__, "Publishing " << metric_list_msg.metrics.size()
                          << " metrics to topic " << kMetricsTopic.c_str());
      metrics_pub->publish(metric_list_msg);
      rclcpp::spin_some(node_handle);
      std::this_thread::sleep_for(std::chrono::seconds(1));
    }
  }

  ros_monitoring_msgs::msg::MetricData BasicMetricData()
  {
    ros_monitoring_msgs::msg::MetricData metric_data = ros_monitoring_msgs::msg::MetricData();
    metric_data.metric_name = kMetricName1;
    metric_data.unit = kMetricUnit1;
    return metric_data;
  }
};

// Test fixture Setup and TearDown
TEST_F(MetricsCollectorFixture, Sanity)
{
  ASSERT_TRUE(true);
}

// Test fixture init
TEST_F(MetricsCollectorFixture, TestInitialize)
{
  std::map<std::string, std::string> metric_dimensions;
  std::vector<std::string> topics = {kMetricsTopic};
  Initialize(metric_dimensions, topics);
}

struct GetMetricDataEpochMillisTestDatum {
  rclcpp::Time input_time;
  int64_t expected_timestamp;
};

class GetMetricDataEpochMillisFixture : public ::testing::TestWithParam<GetMetricDataEpochMillisTestDatum> {};

TEST_P(GetMetricDataEpochMillisFixture, getMetricDataEpochMillisTestOk)
{
  ros_monitoring_msgs::msg::MetricData metric_msg;
  metric_msg.time_stamp = GetParam().input_time;
  EXPECT_EQ(GetParam().expected_timestamp, MetricsCollector::GetMetricDataEpochMillis(metric_msg));
}

const GetMetricDataEpochMillisTestDatum getMetricDataEpochMillisTestData [] = {
  GetMetricDataEpochMillisTestDatum{rclcpp::Time(0, 0), 0},
  GetMetricDataEpochMillisTestDatum{rclcpp::Time(10, 0), 10 * 1000},
  GetMetricDataEpochMillisTestDatum{rclcpp::Time(0, 1), 0},
  GetMetricDataEpochMillisTestDatum{rclcpp::Time(0, 999999), 0},
  GetMetricDataEpochMillisTestDatum{rclcpp::Time(1, 999999), 1000},
  GetMetricDataEpochMillisTestDatum{rclcpp::Time(0, 1000000), 1},
  GetMetricDataEpochMillisTestDatum{rclcpp::Time(1, 1000000), 1001}
};

INSTANTIATE_TEST_CASE_P(getMetricDataEpochMillisTest, GetMetricDataEpochMillisFixture,
                        ::testing::ValuesIn(getMetricDataEpochMillisTestData));

TEST_F(MetricsCollectorFixture, timerCallsMetricManagerService)
{
  std::map<std::string, std::string> metric_dimensions;
  std::vector<std::string> topics = {kMetricsTopic};
  Initialize(metric_dimensions, topics);

  int num_msgs = 3;

  EXPECT_CALL(*metric_service, publishBatchedData())
  .Times(::testing::AnyNumber())
  .WillRepeatedly(::testing::Return(true));

  EXPECT_CALL(*metric_service, batchData(::testing::_))
  .Times(::testing::AtLeast(num_msgs))
  .WillRepeatedly(::testing::Return(true));

  ros_monitoring_msgs::msg::MetricData metric_data = BasicMetricData();
  SendMetricMessages(num_msgs, metric_data);
  for (int i = 0; i < num_msgs; i++) {
    rclcpp::spin_some(node_handle);
    std::this_thread::sleep_for(std::chrono::seconds(1));
  }
}

/**
 * Helper matcher to ensure metric object data, received by the service,
 * matches the data given to the collector.
 */
MATCHER_P(metricsAreEqual, toTest, "")
{
  return arg.metric_name == toTest.metric_name
         && arg.value == toTest.value
         && arg.unit == toTest.unit
         && arg.dimensions == toTest.dimensions
         && arg.storage_resolution == toTest.storage_resolution;
  // timestamp is ignored
}

TEST_F(MetricsCollectorFixture, metricsRecordedNoDimension)
{
  std::map<std::string, std::string> metric_dimensions;
  std::vector<std::string> topics = {kMetricsTopic};
  Initialize(metric_dimensions, topics);

  int num_msgs = 3;

  MetricObject m01 = MetricObject {kMetricName1, 0.0, kMetricUnit1, 1234, std::map<std::string, std::string>(), 60};
  MetricObject m02 = MetricObject {kMetricName1, 1.0, kMetricUnit1, 1234, std::map<std::string, std::string>(), 60};
  MetricObject m03 = MetricObject {kMetricName1, 2.0, kMetricUnit1, 1234, std::map<std::string, std::string>(), 60};

  EXPECT_CALL(*metric_service, publishBatchedData())
  .Times(::testing::AnyNumber())
  .WillRepeatedly(::testing::Return(true));
  {
    ::testing::Sequence rm_seq;
    EXPECT_CALL(*metric_service, batchData(metricsAreEqual(m01)))
    .WillOnce(::testing::Return(true));
    EXPECT_CALL(*metric_service, batchData(metricsAreEqual(m02)))
    .WillOnce(::testing::Return(true));
    EXPECT_CALL(*metric_service, batchData(metricsAreEqual(m03)))
    .WillOnce(::testing::Return(true));
  }

  ros_monitoring_msgs::msg::MetricData metric_data = BasicMetricData();
  SendMetricMessages(num_msgs, metric_data);
  rclcpp::spin_some(node_handle);
}

TEST_F(MetricsCollectorFixture, metricRecordedWithDimension)
{
  int num_msgs = 3;
  const std::string metric_dimension_name = "CWMetricsNodeTestDim1";
  const std::string metric_dimension_value = "CWMetricsNodeTestDim1Value";

  std::vector<std::string> topics = {kMetricsTopic};
  std::map<std::string, std::string> expected_dim;
  expected_dim[metric_dimension_name] = metric_dimension_value;

  MetricObject m01 = MetricObject {kMetricName1, 0.0, kMetricUnit1, 1234, expected_dim, 60};
  MetricObject m02 = MetricObject {kMetricName1, 1.0, kMetricUnit1, 1234, expected_dim, 60};
  MetricObject m03 = MetricObject {kMetricName1, 2.0, kMetricUnit1, 1234, expected_dim, 60};

  EXPECT_CALL(*metric_service, publishBatchedData())
  .Times(::testing::AnyNumber())
  .WillRepeatedly(::testing::Return(true));

  {
    ::testing::Sequence rm_seq;
    EXPECT_CALL(*metric_service, batchData(metricsAreEqual(m01)))
    .WillOnce(::testing::Return(true));
    EXPECT_CALL(*metric_service, batchData(metricsAreEqual(m02)))
    .WillOnce(::testing::Return(true));
    EXPECT_CALL(*metric_service, batchData(metricsAreEqual(m03)))
    .WillOnce(::testing::Return(true));
  }

  Initialize(expected_dim, topics);

  ros_monitoring_msgs::msg::MetricData metric_data = BasicMetricData();
  ros_monitoring_msgs::msg::MetricDimension metric_dimension = ros_monitoring_msgs::msg::MetricDimension();
  metric_dimension.name = metric_dimension_name;
  metric_dimension.value = metric_dimension_value;
  metric_data.dimensions.push_back(metric_dimension);

  SendMetricMessages(num_msgs, metric_data);
  rclcpp::spin_some(node_handle);
}

TEST_F(MetricsCollectorFixture, metricRecordedWithDefaultDimensions)
{
  int num_msgs = 3;
  const std::string metric_dimension_name = "CWMetricsNodeTestDim1";
  const std::string metric_dimension_value = "CWMetricsNodeTestDim1Value";

  std::vector<std::string> topics = {kMetricsTopic};
  std::map<std::string, std::string> expected_dim;
  expected_dim[metric_dimension_name] = metric_dimension_value;

  MetricObject m01 = MetricObject {kMetricName1, 0.0, kMetricUnit1, 1234, expected_dim, 60};
  MetricObject m02 = MetricObject {kMetricName1, 1.0, kMetricUnit1, 1234, expected_dim, 60};
  MetricObject m03 = MetricObject {kMetricName1, 2.0, kMetricUnit1, 1234, expected_dim, 60};

  EXPECT_CALL(*metric_service, publishBatchedData())
  .Times(::testing::AnyNumber())
  .WillRepeatedly(::testing::Return(true));

  {
    ::testing::Sequence rm_seq;
    EXPECT_CALL(*metric_service, batchData(metricsAreEqual(m01)))
    .WillOnce(::testing::Return(true));
    EXPECT_CALL(*metric_service, batchData(metricsAreEqual(m02)))
    .WillOnce(::testing::Return(true));
    EXPECT_CALL(*metric_service, batchData(metricsAreEqual(m03)))
    .WillOnce(::testing::Return(true));
  }

  std::map<std::string, std::string> default_metric_dims;
  default_metric_dims.emplace(metric_dimension_name, metric_dimension_value);
  Initialize(default_metric_dims, topics);

  ros_monitoring_msgs::msg::MetricData metric_data = BasicMetricData();

  SendMetricMessages(num_msgs, metric_data);
  rclcpp::spin_some(node_handle);
}

TEST_F(MetricsCollectorFixture, customTopicsListened)
{
  std::map<std::string, std::string> default_metric_dims;
  std::vector<std::string> topics = {"metrics_topic0", "metrics_topic1"};
  Initialize(default_metric_dims, topics);

  MetricObject m01 = MetricObject {kMetricName1, 0.0, kMetricUnit1, 1234, default_metric_dims, 60};
  MetricObject m02 = MetricObject {kMetricName1, 1.0, kMetricUnit1, 1234, default_metric_dims, 60};

  EXPECT_CALL(*metric_service, publishBatchedData())
  .Times(::testing::AnyNumber())
  .WillRepeatedly(::testing::Return(true));
  EXPECT_CALL(*metric_service, batchData(metricsAreEqual(m01)))
  .Times(1)
  .WillOnce(::testing::Return(true));
  EXPECT_CALL(*metric_service, batchData(metricsAreEqual(m02)))
  .Times(1)
  .WillOnce(::testing::Return(true));

  ros_monitoring_msgs::msg::MetricList metric_list_msg = ros_monitoring_msgs::msg::MetricList();
  ros_monitoring_msgs::msg::MetricData metric_data = BasicMetricData();
  rclcpp::Publisher<ros_monitoring_msgs::msg::MetricList>::SharedPtr metrics_pub0 =
    node_handle->create_publisher<ros_monitoring_msgs::msg::MetricList>(topics[0].c_str(), 1);
  metric_data.value = 0;
  metric_data.time_stamp = node_handle->now();
  metric_list_msg.metrics.clear();
  metric_list_msg.metrics.push_back(metric_data);
  metrics_pub0->publish(metric_list_msg);
  rclcpp::spin_some(node_handle);
  rclcpp::Publisher<ros_monitoring_msgs::msg::MetricList>::SharedPtr metrics_pub1 =
    node_handle->create_publisher<ros_monitoring_msgs::msg::MetricList>(topics[1].c_str(), 1);
  metric_data.value = 1;
  metric_data.time_stamp = node_handle->now();
  metric_list_msg.metrics.clear();
  metric_list_msg.metrics.push_back(metric_data);
  metrics_pub1->publish(metric_list_msg);
  rclcpp::spin_some(node_handle);
  std::this_thread::sleep_for(std::chrono::seconds(1));
  rclcpp::spin_some(node_handle);
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  test_argc = argc;
  test_argv = argv;
  return RUN_ALL_TESTS();
}
