

from datetime import datetime, timezone
import csv
import os
from rosidl_runtime_py.convert import message_to_ordereddict
from ament_index_python.packages import get_package_share_directory
from pathlib import Path

def gps_to_timestamp_string(gps_msg) -> str:
    """
    Convert the UTC fields of a GPSNavPVT message into an ISO8601 timestamp string.

    Expected GPSNavPVT fields:
        gps_msg.utc_year
        gps_msg.utc_month
        gps_msg.utc_day
        gps_msg.utc_hour
        gps_msg.utc_min
        gps_msg.utc_sec   (float, may contain fractional seconds)

    Returns:
        String in the format "YYYY-MM-DDTHH:MM:SS.sssZ"
    """

    # Extract integer and fractional part of seconds
    sec_int = int(gps_msg.utc_sec)
    sec_frac = gps_msg.utc_sec - sec_int

    # Convert fractional part to microseconds
    usec = int(sec_frac * 1e6)

    try:
        dt = datetime(
            year=int(gps_msg.utc_year),
            month=int(gps_msg.utc_month),
            day=int(gps_msg.utc_day),
            hour=int(gps_msg.utc_hour),
            minute=int(gps_msg.utc_min),
            second=sec_int,
            microsecond=usec,
            tzinfo=timezone.utc
        )
    except Exception:
        # If GPS data is corrupted, return an empty string
        return ""

    # Format as ISO8601 in Zulu time
    return dt.isoformat().replace("+00:00", "Z")



def flatten_dict(d, parent_key="", sep="."):
    """
    Flatten nested dicts and lists.
    Example:
        {"trace_1": [1,2], "gps": {"lat": 45}}
    → {"trace_1.0": 1, "trace_1.1": 2, "gps.lat": 45}
    """
    items = {}

    for key, value in d.items():
        new_key = f"{parent_key}{sep}{key}" if parent_key else key

        if isinstance(value, dict):
            items.update(flatten_dict(value, new_key, sep))
        elif isinstance(value, list):
            for idx, item in enumerate(value):
                items[f"{new_key}.{idx}"] = item
        else:
            items[new_key] = value

    return items


def log_msg_to_csv(msg):

    # Convert message → flat dict
    d = message_to_ordereddict(msg)
    flat = flatten_dict(d)

    # Get installed share dir
    share_dir = Path(get_package_share_directory('rf_meas_manager'))

    # share_dir = install/rf_meas_manager/share/rf_meas_manager
    # workspace root = share_dir / ../../../..
    workspace_root = share_dir.parents[3]

    # src/rf_meas_manager/logs
    src_pkg_dir = workspace_root / "src" / "rf_meas_manager"
    log_dir = src_pkg_dir / "logs"
    log_dir.mkdir(parents=True, exist_ok=True)

    # Filename based on msg type
    msg_type = msg.__class__.__name__
    filepath = log_dir / f"{msg_type}.csv"

    # Write header only once
    write_header = not filepath.exists()

    with open(filepath, "a", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=list(flat.keys()))
        if write_header:
            writer.writeheader()
        writer.writerow(flat)