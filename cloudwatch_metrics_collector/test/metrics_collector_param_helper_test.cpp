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

#include <gtest/gtest.h>
#include <gmock/gmock.h>

#include <aws_common/sdk_utils/aws_error.h>
#include <aws_common/sdk_utils/parameter_reader.h>
#include <cloudwatch_metrics_collector/metrics_collector_parameter_helper.hpp>

using namespace Aws::Client;
using namespace Aws::CloudWatchMetrics::Utils;
using ::testing::Eq;
using ::testing::A;
using ::testing::InSequence;
using ::testing::SetArgReferee;
using ::testing::DoAll;
using ::testing::Return;
using ::testing::Invoke;
using Aws::AwsError;

class ParameterReaderMock : public ParameterReaderInterface
{
public:
  MOCK_CONST_METHOD2(ReadParam, Aws::AwsError(const ParameterPath &, bool &));
  MOCK_CONST_METHOD2(ReadParam, Aws::AwsError(const ParameterPath &, int &));
  MOCK_CONST_METHOD2(ReadParam, Aws::AwsError(const ParameterPath &, double &));
  MOCK_CONST_METHOD2(ReadParam, Aws::AwsError(const ParameterPath &, Aws::String &));
  MOCK_CONST_METHOD2(ReadParam, Aws::AwsError(const ParameterPath &, std::string &));
  MOCK_CONST_METHOD2(ReadParam, Aws::AwsError(const ParameterPath &, std::vector<std::string> &));
  MOCK_CONST_METHOD2(ReadParam, Aws::AwsError(const ParameterPath &, std::map<std::string, std::string> &));
};

class MetricsCollectorParamHelperFixture : public ::testing::Test
{
protected:
  std::shared_ptr<ParameterReaderMock> param_reader_ = std::make_shared<ParameterReaderMock>();
};

TEST_F(MetricsCollectorParamHelperFixture, TestReadPublishFrequency)
{
  const int expected_param_value = 42;

  {
    InSequence read_param_seq;

    EXPECT_CALL(*param_reader_, ReadParam(Eq(ParameterPath(kNodeParamPublishFrequencyKey)), A<int &>()))
    .WillOnce(
      Return(AwsError::AWS_ERR_FAILURE)
    );

    EXPECT_CALL(*param_reader_, ReadParam(Eq(ParameterPath(kNodeParamPublishFrequencyKey)), A<int &>()))
    .WillOnce(
      Return(AwsError::AWS_ERR_NOT_FOUND)
    );

    EXPECT_CALL(*param_reader_, ReadParam(Eq(ParameterPath(kNodeParamPublishFrequencyKey)), A<int &>()))
    .WillOnce(DoAll(
      SetArgReferee<1>(expected_param_value), Return(AwsError::AWS_ERR_OK)
    ));
  }

  int param = -1;
  ReadPublishFrequency(param_reader_, param);
  EXPECT_EQ(kNodePublishFrequencyDefaultValue, param);

  param = -1;
  ReadPublishFrequency(param_reader_, param);
  EXPECT_EQ(kNodePublishFrequencyDefaultValue, param);

  param = -1;
  ReadPublishFrequency(param_reader_, param);
  EXPECT_EQ(expected_param_value, param);
}

TEST_F(MetricsCollectorParamHelperFixture, TestReadMetricNamespace)
{
  const std::string expected_param_value = "XYXwEwEtHw";

  {
    InSequence read_param_seq;

    EXPECT_CALL(*param_reader_, ReadParam(Eq(ParameterPath(kNodeParamMetricNamespaceKey)), A<std::string &>()))
    .WillOnce(
      Return(AwsError::AWS_ERR_FAILURE)
    );

    EXPECT_CALL(*param_reader_, ReadParam(Eq(ParameterPath(kNodeParamMetricNamespaceKey)), A<std::string &>()))
    .WillOnce(
      Return(AwsError::AWS_ERR_NOT_FOUND)
    );

    EXPECT_CALL(*param_reader_, ReadParam(Eq(ParameterPath(kNodeParamMetricNamespaceKey)), A<std::string &>()))
    .WillOnce(DoAll(
      SetArgReferee<1>(expected_param_value), Return(AwsError::AWS_ERR_OK)
    ));
  }

  std::string param = "";
  ReadMetricNamespace(param_reader_, param);
  EXPECT_STREQ(kNodeDefaultMetricNamespace, param.c_str());

  param = "";
  ReadMetricNamespace(param_reader_, param);
  EXPECT_STREQ(kNodeDefaultMetricNamespace, param.c_str());

  param = "";
  ReadMetricNamespace(param_reader_, param);
  EXPECT_STREQ(expected_param_value.c_str(), param.c_str());
}

TEST_F(MetricsCollectorParamHelperFixture, TestReadMetricDimensions)
{
  // TODO
}

TEST_F(MetricsCollectorParamHelperFixture, TestReadStorageResolution)
{
  const int invalid_param_value = 2;
  const int expected_param_value = 1;

  {
    InSequence read_param_seq;

    EXPECT_CALL(*param_reader_, ReadParam(Eq(ParameterPath(kNodeParamMetricDatumStorageResolutionKey)), A<int &>()))
    .WillOnce(
      Return(AwsError::AWS_ERR_FAILURE)
    );

    EXPECT_CALL(*param_reader_, ReadParam(Eq(ParameterPath(kNodeParamMetricDatumStorageResolutionKey)), A<int &>()))
    .WillOnce(
      Return(AwsError::AWS_ERR_NOT_FOUND)
    );

    EXPECT_CALL(*param_reader_, ReadParam(Eq(ParameterPath(kNodeParamMetricDatumStorageResolutionKey)), A<int &>()))
    .WillOnce(DoAll(
      SetArgReferee<1>(invalid_param_value), Return(AwsError::AWS_ERR_OK)
    ));

    EXPECT_CALL(*param_reader_, ReadParam(Eq(ParameterPath(kNodeParamMetricDatumStorageResolutionKey)), A<int &>()))
    .WillOnce(DoAll(
      SetArgReferee<1>(expected_param_value), Return(AwsError::AWS_ERR_OK)
    ));
  }

  int param = -1;
  ReadStorageResolution(param_reader_, param);
  EXPECT_EQ(kNodeDefaultMetricDatumStorageResolution, param);

  param = -1;
  ReadStorageResolution(param_reader_, param);
  EXPECT_EQ(kNodeDefaultMetricDatumStorageResolution, param);

  param = -1;
  ReadStorageResolution(param_reader_, param);
  EXPECT_EQ(kNodeDefaultMetricDatumStorageResolution, param);

  param = -1;
  ReadStorageResolution(param_reader_, param);
  EXPECT_EQ(expected_param_value, param);
}

TEST_F(MetricsCollectorParamHelperFixture, TestReadTopics)
{
  const std::vector<std::string> expected_param_value = {"NLpSNMmil4", "oYazRO4x7O"};

  auto MockReadParamAddStringToVector =
    [&expected_param_value](const ParameterPath & /*param_path*/, std::vector<std::string> & out)
    {
      out.emplace_back(expected_param_value[0]);
      out.emplace_back(expected_param_value[1]);
      return AwsError::AWS_ERR_OK;
    };

  {
    InSequence read_param_seq;

    EXPECT_CALL(*param_reader_, ReadParam(Eq(ParameterPath(kNodeParamMonitoringTopicsListKey)), A<std::vector<std::string> &>()))
    .WillOnce(
      Return(AwsError::AWS_ERR_FAILURE)
    );

    EXPECT_CALL(*param_reader_, ReadParam(Eq(ParameterPath(kNodeParamMonitoringTopicsListKey)), A<std::vector<std::string> &>()))
    .WillOnce(
      Return(AwsError::AWS_ERR_NOT_FOUND)
    );

    EXPECT_CALL(*param_reader_, ReadParam(Eq(ParameterPath(kNodeParamMonitoringTopicsListKey)), A<std::vector<std::string> &>()))
    .WillOnce(
      Invoke(MockReadParamAddStringToVector)
    );
  }

  std::vector<std::string> param;
  ReadTopics(param_reader_, param);
  ASSERT_EQ(1u, param.size());
  EXPECT_EQ(kNodeDefaultMetricsTopic, param[0]);

  param.clear();
  ReadTopics(param_reader_, param);
  ASSERT_EQ(1u, param.size());
  EXPECT_EQ(kNodeDefaultMetricsTopic, param[0]);

  param.clear();
  ReadTopics(param_reader_, param);
  ASSERT_EQ(2u, param.size());
  EXPECT_EQ(expected_param_value[0], param[0]);
  EXPECT_EQ(expected_param_value[1], param[1]);
}

TEST_F(MetricsCollectorParamHelperFixture, TestReadCloudWatchOptions)
{
  // TODO
}

TEST_F(MetricsCollectorParamHelperFixture, TestReadUploaderOptions)
{
  // TODO
}

TEST_F(MetricsCollectorParamHelperFixture, TestReadFileManagerStrategyOptions)
{
  // TODO
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
