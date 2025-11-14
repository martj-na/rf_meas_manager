# rf_meas_manager/rf_meas_manager/rf_meas_manager.py

from rf_meas_manager_interfaces.msg import ChannelPower, ZeroSpan
from rf_meas_manager.rf_meas_manager_utils import gps_to_timestamp_string

class RFMeasManager:
    def __init__(self):
        """Initialize processor state."""
        self.last_gps = None

    def update_gps(self, gps_msg):
        """
        Store the latest valid GPS message.
        This should be called on each GPSNavPVT callback.
        """
        self.last_gps = gps_msg

    def build_chpow_message(self, raw_msg):
        """
        Build a ChannelPower message using:
        - the latest GPS message
        - the raw Float64MultiArray from the Anritsu driver

        Expected raw_msg.data format:
            raw_msg.data[0] = channel power (dBm)
            raw_msg.data[1] = PSD (dBm/Hz)

        Returns:
            ChannelPower msg or None if GPS is missing or raw is invalid.
        """
        # GPS must be available
        if self.last_gps is None:
            return None

        # Extract raw measurement values from the analyzer
        try:
            ch_power = float(raw_msg.data[0])
            psd = float(raw_msg.data[1])
        except (IndexError, ValueError):
            return None

        # Convert GPS UTC fields to ISO8601 timestamp string
        utc_timestamp = gps_to_timestamp_string(self.last_gps)

        # Build final ChannelPower message
        msg = ChannelPower()
        msg.utc_timestamp = utc_timestamp
        msg.latitude = float(self.last_gps.pos_latitude)
        msg.longitude = float(self.last_gps.pos_longitude)
        msg.height = float(self.last_gps.pos_height)
        msg.height_msl = float(self.last_gps.pos_height_msl)
        msg.ch_power_dbm = ch_power
        msg.psd_dbm_hz = psd

        return msg

    
    def build_zerospan_message(self, raw_msg):
        """
        Build a ZeroSpan message from a 6×501 Float64MultiArray published by the driver.

        Requirements:
        - Exactly 6 traces
        - Exactly 501 points per trace
        - GPS must be available

        Returns ZeroSpan or None if invalid.
        """

        # 1) GPS must be available
        if self.last_gps is None:
            return None

        # 2) Validate layout
        if len(raw_msg.layout.dim) < 2:
            return None

        num_traces = raw_msg.layout.dim[0].size
        points_per_trace = raw_msg.layout.dim[1].size

        # 3) Rigid validation: must be exactly 6 traces
        if num_traces != 6:
            return None

        # 4) Rigid validation: expected length
        expected_len = num_traces * points_per_trace
        if len(raw_msg.data) != expected_len:
            return None

        # 5) Build list of traces
        data = raw_msg.data
        traces = []
        for i in range(num_traces):
            start = i * points_per_trace
            end = start + points_per_trace
            segment = data[start:end]

            # Must be EXACT size
            if len(segment) != points_per_trace:
                return None

            traces.append([float(v) for v in segment])  # ROS cast to float32 later

        # 6) Build ZeroSpan message
        msg = ZeroSpan()
        msg.utc_timestamp = gps_to_timestamp_string(self.last_gps)
        msg.latitude      = float(self.last_gps.pos_latitude)
        msg.longitude     = float(self.last_gps.pos_longitude)
        msg.height        = float(self.last_gps.pos_height)
        msg.height_msl    = float(self.last_gps.pos_height_msl)

        msg.trace_1 = traces[0]
        msg.trace_2 = traces[1]
        msg.trace_3 = traces[2]
        msg.trace_4 = traces[3]
        msg.trace_5 = traces[4]
        msg.trace_6 = traces[5]

        return msg
            