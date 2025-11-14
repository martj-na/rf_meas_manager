

# Copyright 2025 dotX Automation s.r.l.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

#!/usr/bin/env python

import sys
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rf_meas_manager.rf_meas_manager_node import RFMeasManagerNode

def main():
    # Initialize ROS 2 context and node
    rclpy.init(args=sys.argv)
    rf_meas_manager_node = RFMeasManagerNode(node_name='rf_meas_manager', verbose=True)

    executor = SingleThreadedExecutor()
    executor.add_node(rf_meas_manager_node)

    # Run the node
    try:
        executor.spin()
    finally:
        executor.shutdown()
        rf_meas_manager_node.cleanup()
        rf_meas_manager_node.destroy_node()

if __name__ == '__main__':
    main()
