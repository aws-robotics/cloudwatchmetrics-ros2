^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package cloudwatch_metrics_collector
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

3.0.0 (2019-09-06)
------------------
* Bump version to 3.0.0 and add missing dependencies
* Fix default metric options
  - Fix bug where default metric storage directory, file prefix etc were
  being set to the same as logs causing issues when running both
  cloudwatch logs and metrics at the same time.
* Merge pull request `#7 <https://github.com/aws-robotics/cloudwatchmetrics-ros2/issues/7>`_ from aws-robotics/rename-launch-file
  Rename launch file from _launch.py to .launch.py
* Rename cloudwatch_metrics_collector_launch.py to cloudwatch_metrics_collector.launch.py
* Add Unit Tests (`#6 <https://github.com/aws-robotics/cloudwatchmetrics-ros2/issues/6>`_)
  * add unit tests
  Signed-off-by: Miaofei <miaofei@amazon.com>
  * Enable travis tests
* Adds launch file (`#3 <https://github.com/aws-robotics/cloudwatchmetrics-ros2/issues/3>`_)
  * Add launch file
* Initial source code migration (`#1 <https://github.com/aws-robotics/cloudwatchmetrics-ros2/issues/1>`_)
  * Initial source code migration
  Signed-off-by: Ryan Newell <ryanewel@amazon.com>
  * Adds README and sample configuration
  Signed-off-by: Ryan Newell <ryanewel@amazon.com>
  * Adds .rosinstall
  * Remove dependency on parameter reader from metrics collector
  * Fix typo in README
* Contributors: Avishay Alon, M. M, Tim Robinson, ryanewel
