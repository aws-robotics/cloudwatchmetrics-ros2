^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package cloudwatch_metrics_collector
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

3.0.1 (2020-03-31)
------------------
* Bump version to 3.0.1 (`#18 <https://github.com/aws-robotics/cloudwatchmetrics-ros2/issues/18>`_)
* Add file management include to tests (`#19 <https://github.com/aws-robotics/cloudwatchmetrics-ros2/issues/19>`_)
* Fix linting issues found by clang-tidy 6.0 (`#15 <https://github.com/aws-robotics/cloudwatchmetrics-ros2/issues/15>`_)
  * clang-tidy fixes
  * clang-tidy linting issues fixed manually
  * optimize ReadTopics()
  * add more unit tests
* Revert "Add support for metrics_statistics_msgs (`#14 <https://github.com/aws-robotics/cloudwatchmetrics-ros2/issues/14>`_)" (`#16 <https://github.com/aws-robotics/cloudwatchmetrics-ros2/issues/16>`_)
  This reverts commit 097032087a027bfa4d218358f317b70453346688.
* Add support for metrics_statistics_msgs (`#14 <https://github.com/aws-robotics/cloudwatchmetrics-ros2/issues/14>`_)
  * add support for metrics_statistics_msgs
  * enable subscription callback
  * move to using two distinct list of topics
  * address some PR comments
  * address more PR comments
  * address feedback regarding MetricsMessage to MetricsObject mapping
  * add unit tests for MetricsCollector
  * update MetricsMessage to MetricsObject mapping
  * address comments regarding unit tests
  * updating README.md
* Contributors: Miaofei Mei, Ryan Newell

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
