
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

from threading import Thread


from dua_node_py.dua_node import NodeBase
import dua_qos_py.dua_qos_besteffort as dua_qos_besteffort
import dua_qos_py.dua_qos_reliable as dua_qos_reliable

from std_msgs.msg import String, Empty


from std_srvs.srv import SetBool

from dua_hardware_interfaces.msg import GPSNavPVT
from std_msgs.msg import Float64MultiArray
from rf_meas_manager_interfaces.msg import ChannelPower, ZeroSpan

from rf_meas_manager.rf_meas_manager_class import RFMeasManager

from typing import Any

from functools import partial


class RFMeasManagerNode(NodeBase):
    """
    RF Measurements Manager node class.
    """
    def __init__(self, node_name: str, verbose: bool) -> None:
        """
        Constructor.

        :param node_name: Name of the node.
        :param verbose: Verbosity flag.
        """
        super().__init__(node_name, verbose)

        self.parser = RFMeasManager()

        self.get_logger().warn('RF Measurements Manager initialized')

    def __del__(self) -> None:
        """
        Destructor.
        """
        self.get_logger().warn('RF Measurements Manager destroyed')

 

    def init_publishers(self) -> None:
        """
        Init publishers.
        """
        # Publisher for Channel Power data
        self.publisher_chpow = self.dua_create_publisher(
            ChannelPower,
            '/channel_power')
         # Publisher for Channel Power data
        self.publisher_zerospan = self.dua_create_publisher(
            ZeroSpan,
            '/zero_span')

       

    def init_subscribers(self) -> None:
        """
        Init subscribers.
        """
        self.dua_create_subscription(
            GPSNavPVT,
            self._topic_gnss,
            self.gnss_callback
        )
        self.dua_create_subscription(
            Float64MultiArray,
            self._topic_chpow_raw,
            self.chpow_callback
        )
        self.dua_create_subscription(
            Float64MultiArray,
            self._topic_zerospan_raw,
            self.zerospan_callback
        )
       

    

    def gnss_callback(self, gps_msg) -> None:
        """
        Callback for GPS messages.

        :param msg: GPSNavPvt message.
        """
        self.parser.update_gps(gps_msg)



    def chpow_callback(self, raw_msg) -> None:
        """
        Callback for raw Channel Power messages coming from the driver.
        :param msg: Float64MultiArray message.
        """
        try:
            msg = self.parser.build_chpow_message(raw_msg)
            if msg is None:
                return

            self.publisher_chpow.publish(msg)

            if self._verbose:
                self.get_logger().info("Published ChannelPower message")

        except Exception as e:
            self.get_logger().error(f"Error in chpow_callback: {e}")

    def zerospan_callback(self, raw_msg) -> None:
        """
        Callback for raw Zero Span traces coming from the driver.
        :param raw_msg: Float64MultiArray message (6 x N matrix).
        """
        try:
            msg = self.parser.build_zerospan_message(raw_msg)
            if msg is None:
                return

            self.publisher_zerospan.publish(msg)

            if self._verbose:
                self.get_logger().info("Published ZeroSpan message")

        except Exception as e:
            self.get_logger().error(f"Error in zerospan_callback: {e}")
