#!/usr/bin/env python3
"""
Make a rosbag with /visual_slam/pose from a TXT file.

Each non-comment line in the input must have 8 fields:
timestamp  tx  ty  tz  qx  qy  qz  qw

- timestamp: UNIX epoch (sec as float), or ms/us/ns as integer
- tx ty tz: translation (meters)
- qx qy qz qw: quaternion (unit)

Example line:
1695926400.050  1.23  -0.5  0.02   0.0  0.0  0.7071068  0.7071068
"""

import argparse
import math
import sys

import rospy
import rosbag
from geometry_msgs.msg import PoseStamped

def parse_args():
    p = argparse.ArgumentParser(description="Write /visual_slam/pose rosbag from TXT")
    p.add_argument("input_txt", help="Input TXT file")
    p.add_argument("output_bag", help="Output rosbag file, e.g., poses.bag")
    p.add_argument("--frame-id", default="orbslam_map", help="Header frame_id (default: map)")
    p.add_argument("--skip-invalid", action="store_true",
                   help="Skip lines with wrong field counts or NaNs (default: stop on error)")
    return p.parse_args()

def normalize_epoch_to_sec(t_raw):
    """
    Convert a numeric epoch value to seconds (float).
    Accepts:
      - seconds (e.g., 1695926400.123)
      - integer ms/us/ns (>= 10^11)
    """
    # If it's a float already, assume seconds
    if isinstance(t_raw, float):
        return t_raw

    # If it's str, try int then float
    try:
        # Try integer first
        ival = int(t_raw)
        # Heuristics by magnitude
        # ~1e9 = seconds (current epoch ~ 1.7e9 in 2025)
        # ~1e12 = milliseconds
        # ~1e15 = microseconds
        # ~1e18 = nanoseconds
        if ival >= 10**17:     # ns
            return ival / 1e9
        elif ival >= 10**14:   # us
            return ival / 1e6
        elif ival >= 10**11:   # ms
            return ival / 1e3
        else:                  # seconds (int)
            return float(ival)
    except ValueError:
        # Not an int; try float seconds
        return float(t_raw)

def parse_line(line, line_num):
    # Strip comments and whitespace
    line = line.strip()
    if not line or line.startswith("#"):
        return None  # signal to skip

    parts = line.split()
    if len(parts) != 8:
        raise ValueError(f"Line {line_num}: expected 8 columns, got {len(parts)} -> {parts}")

    t_raw, tx, ty, tz, qx, qy, qz, qw = parts

    t_sec = normalize_epoch_to_sec(t_raw)

    # Convert pose fields
    tx, ty, tz = float(tx), float(ty), float(tz)
    qx, qy, qz, qw = float(qx), float(qy), float(qz), float(qw)

    # Optional: sanity-check quaternion normalization (tolerant)
    q_norm = math.sqrt(qx*qx + qy*qy + qz*qz + qw*qw)
    if not (0.5 <= q_norm <= 1.5):
        # warn rather than fail; user may prefer --skip-invalid
        rospy.logwarn_once("Quaternion norm seems off (%.6f). Ensure your inputs are normalized.", q_norm)

    return t_sec, (tx, ty, tz, qx, qy, qz, qw)

def main():
    args = parse_args()

    # Initialize rospy (anonymous ok; we don't publish, only need Time conversions)
    if not rospy.core.is_initialized():
        rospy.init_node("txt_to_bag_pose_converter", anonymous=True, disable_signals=True)

    topic = "/visual_slam/pose"

    n_written = 0
    n_skipped = 0

    try:
        bag = rosbag.Bag(args.output_bag, "w")
    except Exception as e:
        print(f"Failed to open bag for writing: {e}", file=sys.stderr)
        sys.exit(1)

    try:
        with open(args.input_txt, "r") as f:
            for i, raw in enumerate(f, start=1):
                try:
                    parsed = parse_line(raw, i)
                    if parsed is None:
                        continue  # comment/blank

                    t_sec, (tx, ty, tz, qx, qy, qz, qw) = parsed

                    msg = PoseStamped()
                    msg.header.frame_id = args.frame_id
                    msg.header.stamp = rospy.Time.from_sec(t_sec)

                    msg.pose.position.x = tx
                    msg.pose.position.y = ty
                    msg.pose.position.z = tz
                    msg.pose.orientation.x = qx
                    msg.pose.orientation.y = qy
                    msg.pose.orientation.z = qz
                    msg.pose.orientation.w = qw

                    # Write with the same timestamp as in the header
                    bag.write(topic, msg, t=msg.header.stamp)
                    n_written += 1

                except Exception as ex:
                    if args.skip_invalid:
                        n_skipped += 1
                        rospy.logwarn("Skipping line %d: %s", i, ex)
                        continue
                    else:
                        raise

    except FileNotFoundError:
        print(f"Input file not found: {args.input_txt}", file=sys.stderr)
        bag.close()
        sys.exit(1)
    except Exception as e:
        bag.close()
        raise
    finally:
        bag.close()

    print(f"Done. Wrote {n_written} PoseStamped messages to '{args.output_bag}' on topic '{topic}'.")
    if n_skipped:
        print(f"Skipped {n_skipped} malformed lines.")

if __name__ == "__main__":
    main()
