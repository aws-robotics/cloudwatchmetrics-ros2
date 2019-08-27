# Copyright 2019 Amazon.com, Inc. or its affiliates. All Rights Reserved.
# 
# Licensed under the Apache License, Version 2.0 (the "License").
# You may not use this file except in compliance with the License.
# A copy of the License is located at
#
#  http://aws.amazon.com/apache2.0
#
# or in the "license" file accompanying this file. This file is distributed
# on an "AS IS" BASIS, WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either
# express or implied. See the License for the specific language governing
# permissions and limitations under the License.
import os

from ament_index_python.packages import get_package_share_directory
import launch
import launch_ros.actions

# Argument names
NODE_NAME = "node_name"
CONFIG = "config"

# AWS RoboMaker Environment Variables
class RoboMakerEnvVariables():
  CAPABILITY = "AWS_ROBOMAKER_CAPABILITY"
  ROBOT_NAME = "AWS_ROBOMAKER_ROBOT_NAME"
  SIMULATION_JOB_ID = "AWS_ROBOMAKER_SIMULATION_JOB_ID"

def generate_launch_description():

  # Default to included config file
  default_config = os.path.join(get_package_share_directory('cloudwatch_metrics_collector'),
    'config', 'sample_configuration.yaml')

  
  aws_robomaker_metric_namespace = None
  aws_default_metric_dimensions = None
  if RoboMakerEnvVariables.CAPABILITY in os.environ:
    aws_robomaker_metric_namespace = os.environ[RoboMakerEnvVariables.CAPABILITY]
    # If this is the RoboMaker Fleet Management environment then add the robot
    # name as a default dimension
    if os.environ[RoboMakerEnvVariables.CAPABILITY] == "AWSRoboMakerFleetManagement" and RoboMakerEnvVariables.ROBOT_NAME in os.environ:
      aws_default_metric_dimensions = "RobotName:" + os.environ.get(RoboMakerEnvVariables.ROBOT_NAME)
    # If this is the RoboMaker Simulation envrionment then add the simulation job
    # ID as a default dimension
    if os.environ[RoboMakerEnvVariables.CAPABILITY] == "AWSRoboMakerSimulation" and RoboMakerEnvVariables.SIMULATION_JOB_ID in os.environ:
       aws_default_metric_dimensions = "SimulationJobId:" + os.environ.get(RoboMakerEnvVariables.SIMULATION_JOB_ID)

  
      
  parameters = [launch.substitutions.LaunchConfiguration(CONFIG), ]
  if aws_robomaker_metric_namespace:
    parameters.append({"aws_metrics_namespace": aws_robomaker_metric_namespace})
  if aws_default_metric_dimensions:
    parameters.append({"aws_default_metric_dimensions": aws_default_metric_dimensions})
      


  ld = launch.LaunchDescription([
    launch.actions.DeclareLaunchArgument(
      NODE_NAME,
      default_value="cloudwatch_metrics_collector",
    ),
    launch.actions.DeclareLaunchArgument(
      CONFIG,
      default_value=default_config
    )
   ])
  encoder_node = launch_ros.actions.Node(
    package="cloudwatch_metrics_collector",
    node_executable="cloudwatch_metrics_collector",
    node_name=launch.substitutions.LaunchConfiguration(NODE_NAME),
    parameters=parameters
  )

  ld.add_action(encoder_node)

  return ld


if __name__ == "__main__":
  generate_launch_description()
