

from datetime import datetime, timezone

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