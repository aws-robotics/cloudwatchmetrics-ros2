## Cloudwatchmetrics Ros2
**Note: this repository is under active development. The package provided here is a release candidate; the API may change without notice and no support is provided for it at the moment.**

**Note: this repository is under active development. The package provided here is a release candidate; the API may change without notice and no support is provided for it at the moment.**

## Overview
The `cloudwatch_metrics_collector` ROS Node publishes your robot's metrics to the cloud to enable you to easily track the health of a fleet with the use of automated monitoring and actions for when a robot's metrics show abnormalities. You can easily track historic trends and profile behavior such as resource usage. Out of the box it provides a ROS interface to take a ROS message defining a metric and publish it to Amazon CloudWatch Metrics. The only configuration required to get started is setting up AWS Credentials and Permissions for your robot.The CloudWatch Metrics Node can be used with existing ROS Nodes that publish metric messages or with your own nodes if you instrument them to publish your own custom metrics.

**Amazon CloudWatch Metrics Summary**: Amazon CloudWatch is a monitoring and management service built for developers, system operators, site reliability engineers (SRE), and IT managers. CloudWatch provides you with data and actionable insights to monitor your applications, understand and respond to system-wide performance changes, optimize resource utilization, and get a unified view of operational health. CloudWatch collects monitoring and operational data in the form of logs, metrics, and events, providing you with a unified view of AWS resources, applications and services that run on AWS, and on-premises servers. You can use CloudWatch to set high resolution alarms, visualize logs and metrics side by side, take automated actions, troubleshoot issues, and discover insights to optimize your applications, and ensure they are running smoothly.

**Keywords**: ROS, ROS2, AWS, CloudWatch, Metrics

### License
The source code is released under an [Apache 2.0].

**Author**: AWS RoboMaker<br/>
**Affiliation**: [Amazon Web Services (AWS)]<br/>
**Maintainer**: AWS RoboMaker, ros-contributions@amazon.com

### Supported ROS Distributions
- Dashing 

### Build status
* Travis CI:
    * "master" branch [![Build Status](https://travis-ci.org/aws-robotics/cloudwatchmetrics-ros2.svg?branch=master)](https://travis-ci.org/aws-robotics/cloudwatchmetrics-ros2/branches)
    * "release-latest" branch [![Build Status](https://travis-ci.org/aws-robotics/cloudwatchmetrics-ros2.svg?branch=release-latest)](https://travis-ci.org/aws-robotics/cloudwatchmetrics-ros2/branches)
* ROS build farm: **Not Yet Released**


## Installation

### AWS Credentials
You will need to create an AWS Account and configure the credentials to be able to communicate with AWS services. You may find [AWS Configuration and Credential Files] helpful. Specifying AWS [credentials by setting environment variables](https://docs.aws.amazon.com/cli/latest/userguide/cli-environment.html) is not supported.

This node will require the following AWS account IAM role permissions:
- `cloudwatch:PutMetricData`

### Binaries
**Not available in apt yet**

On Ubuntu you can install the latest version of this package using the following command

        sudo apt-get update
        sudo apt-get install -y ros-$ROS_DISTRO-cloudwatch-metrics-collector

### Building from Source

To build from source you'll need to create a new workspace, clone and checkout the latest release branch of this repository, install all the dependencies, and compile. If you need the latest development features you can clone from the `master` branch instead of the latest release branch. While we guarantee the release branches are stable, __the `master` should be considered to have an unstable build__ due to ongoing development. 

- Create a ROS workspace and a source directory

        mkdir -p ~/ros-workspace/src

- Clone the package into the source directory . 

        cd ~/ros-workspace/src
        git clone https://github.com/aws-robotics/cloudwatchmetrics-ros2.git

- Fetch unreleased dependencies from github

        cd ~/ros-workspace 
        cp src/cloudwatchmetrics-ros2/.rosinstall.master .rosinstall
        rosws update

- Install dependencies

        cd ~/ros-workspace 
        sudo apt-get update && rosdep update
        rosdep install --from-paths src --ignore-src -r -y
        
- Build the packages

        cd ~/ros-workspace && colcon build

- Configure ROS library Path

        source ~/ros-workspace/install/setup.bash

- Build and run the unit tests

        colcon build
        colcon test && colcon test-result --all


## Launch Files

The launch file uses the following arguments:

| Arg Name | Description |
| --------- | ------------ |
| node_name | (optional) The name the metrics node should be launched with. If not provided the node will default to "cloudwatch_metrics_collector" |
| config_file | (optional) A path to a config file. If provided the launch file will to load the configuration into the private namespace of the node, otherwise defaults to the sample config file. |

An example launch file called `cloudwatch_metrics_collector_launch.py` is included in this project that gives an example of how you can include this node in your project and provide it with arguments.


## Usage

### Run the node
- Launch the node by itself
  - ``ros2 run cloudwatch_metrics_collector cloudwatch_metrics_collector __params:=`ros2 pkg prefix cloudwatch_metrics_collector`/share/cloudwatch_metrics_collector/config/sample_configuration.yaml``
- With launch file using parameters in .yaml format (example provided)
  - `ros2 launch cloudwatch_metrics_collector cloudwatch_metrics_collector_launch.py` 

### Send a test metric 
- `timestamp=$(date +%s); ros2 topic pub /metrics ros_monitoring_msgs/MetricList "{metrics: [{header:{stamp:{sec: ${timestamp}, nanosec: 0}} , metric_name: 'cw_offline_metric', unit: 'Count', value: 1.0, time_stamp: {sec: ${timestamp}, nanosec: 0}, dimensions: [{name: 'example_dimension', value: 'example_value'}]}]}"`


## Configuration File and Parameters
An example configuration file named `sample_configuration.yaml` is provided that contains a detailed example configuration for the Node.

| Parameter Name | Description | Type | Default |
| ------------- | -----------------------------------------------------------| ------------- | ------------ |
| aws_metrics_namespace | If provided it will set the namespace for all metrics provided by this node to the provided value. If the node is running on AWS RoboMaker then the provided launch file will ignore this parameter in favor of the namespace specified by the AWS RoboMaker ecosystem | *string* | ROS |
| aws_monitored_metric_topics | An optional list of topics to listen to. If not provided or is empty the node will just listen on the global "metrics" topic. If this list is not empty then the node will not subscribe to the "metrics" topic and will only subscribe to the topics in the list. | *array of strings* | metrics |
| storage_directory | The location where all offline metrics will be stored | *string* | ~/.ros/cwmetrics/ |
| storage_limit | The maximum size of all offline storage files in KB. Once this limit is reached offline logs will start to be deleted oldest first. | *int* | 1048576 |
| aws_client_configuration | If given the node will load the provided configuration when initializing the client. If a specific configuration setting is not included in the map then it will search up the namespace hierarchy for an 'aws_client_configuration' map that contains the field. In this way, a global configuration can be provided for all AWS nodes with only specific values overridden for a specific Node instance if needed | *map* | |
| storage_resolution | The storage resolution level for presenting metrics in CloudWatch. For more information, see [high-resolution-metrics](http://docs.aws.amazon.com/AmazonCloudWatch/latest/monitoring/publishingMetrics.html). | *int* | 60 |


### Advanced Configuration Parameters
Most users won't need to touch these parameters, they are useful if you want fine grained control over how your metrics are stored offline and uploaded to CloudWatch. 

| Parameter Name | Description | Type | Default |
| ------------- | -----------------------------------------------------------| ------------- | ------------ |
| batch_max_queue_size | The maximum number metrics items to add to the CloudWatch upload queue before they start to be written to disk | *int* | 1024 |
| batch_trigger_publish_size | Only publish metrics to CloudWatch when there are this many items in the queue. When this is set the publishing of metrics on a constant timer is disabled. This must be smaller than batch_max_queue_size. Metrics uploaded from offline storage are not affected by this. | *int* | *unset* |
| file_max_queue_size | The max number of batches in the queue, each of size file_upload_batch_size, when reading and uploading from offline storage files | *int* | 5 |
| file_upload_batch_size | The size of each batch of metrics in the queue, when reading and uploading from offline storage files | *int* | 50 |
| file_prefix | A prefix to add to each offline storage file so they're easier to identify later | *string* | cwmetric |
| file_extension | The extension for all offline storage files | *string* | .log |
| maximum_file_size | The maximum size each offline storage file in KB | *int* | 1024 |
| stream_max_queue_size | The maximum number of batches in the queue to stream to CloudWatch. If this queue is full subsequent batches of metrics will be written to disk. | *int* | 3 |


## Node

### cloudwatch_metrics_collector
Sends metrics in a ROS system to AWS CloudWatch Metrics service.

#### Subscribed Topics
- **`Configurable (default="/metrics")`**

  The node can subscribe to a configurable list of topic names. If no list in provided then it will default to subscribing to a global topic names `/metrics`.<br/>
  Message Type: `ros_monitoring_msgs/MetricList`

#### Published Topics
None

#### Services
None


## Bugs & Feature Requests
Please contact the team directly if you would like to request a feature.

Please report bugs in [Issue Tracker].


[Amazon Web Services (AWS)]: https://aws.amazon.com/
[Apache 2.0]: https://aws.amazon.com/apache-2-0/
[AWS Configuration and Credential Files]: https://docs.aws.amazon.com/cli/latest/userguide/cli-config-files.html
[high-resolution-metrics]: https://docs.aws.amazon.com/AmazonCloudWatch/latest/monitoring/publishingMetrics.html#high-resolution-metrics
[Issue Tracker]: https://github.com/aws-robotics/cloudwatchmetrics-ros2/issues
[ROS]: http://www.ros.org
