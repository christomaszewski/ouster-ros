"""Launch rosbag playback with explicit, namespace-aware raw packet QoS."""

import json
import math
from pathlib import Path
import re
import tempfile


def resolve_namespace(parent, requested):
    """Follow ROS namespace composition, including an absolute override."""
    if requested.startswith("/"):
        return requested
    return "/".join(part.strip("/") for part in (parent, requested)
                    if part.strip("/"))


def prepare_replay(bag_file, namespace, depth, reliability, rate, delay, loop,
                   override_path=""):
    """Return (command, delay, profile path, generated) without starting ROS."""
    depth = int(depth)
    if not 1 <= depth <= 65536:
        raise ValueError("raw_packet_qos_depth must be in [1, 65536]")
    if reliability not in ("reliable", "best_effort"):
        raise ValueError("raw_packet_qos_reliability must be reliable or best_effort")
    rate, delay = float(rate), float(delay)
    if not math.isfinite(rate) or rate <= 0:
        raise ValueError("play_rate must be finite and positive")
    if not math.isfinite(delay) or delay < 0:
        raise ValueError("play_delay must be finite and nonnegative")
    loop = str(loop).lower()
    if loop not in ("true", "false", "1", "0"):
        raise ValueError("loop must be true or false")
    namespace = namespace.strip("/")
    if namespace and not re.fullmatch(r"[A-Za-z_][A-Za-z0-9_]*(/[A-Za-z_][A-Za-z0-9_]*)*", namespace):
        raise ValueError("ouster_ns must be a valid ROS namespace")
    target = "/" + namespace if namespace else ""
    aliases = sorted({"/os_node", "/ouster", target})

    generated = not override_path
    if override_path:
        profile_path = Path(override_path).expanduser().resolve()
        # Fail before launching rosbag, without reading or replacing its content.
        with profile_path.open("rb"):
            pass
    else:
        profiles = {}
        for prefix in aliases:
            profiles[prefix + "/metadata"] = {
                "history": "keep_last", "depth": 1,
                "reliability": "reliable", "durability": "transient_local",
            }
            for topic in ("lidar_packets", "imu_packets"):
                profiles[prefix + "/" + topic] = {
                    "history": "keep_last", "depth": depth,
                    "reliability": reliability, "durability": "volatile",
                }
        # JSON is a YAML subset and needs no additional Python dependency.
        with tempfile.NamedTemporaryFile(
                mode="w", prefix="ouster-replay-qos-", suffix=".json",
                delete=False) as profile:
            json.dump(profiles, profile)
            profile_path = Path(profile.name)

    command = ["ros2", "bag", "play", bag_file, "--clock", "--rate", str(rate),
               "--qos-profile-overrides-path", str(profile_path)]
    if loop in ("true", "1"):
        command.append("--loop")
    remaps = [prefix + "/" + topic + ":=" + target + "/" + topic
              for prefix in aliases if prefix != target
              for topic in ("metadata", "lidar_packets", "imu_packets")]
    if remaps:
        command.extend(["--remap", *remaps])
    return command, delay, profile_path, generated


def cleanup_profile(profile_path, generated):
    """Remove only a profile created by this launch, including repeated cleanup."""
    if generated:
        profile_path.unlink(missing_ok=True)


def launch_player(context):
    from launch.actions import ExecuteProcess, OpaqueFunction, RegisterEventHandler, TimerAction
    from launch.event_handlers import OnProcessExit, OnShutdown
    from launch.substitutions import LaunchConfiguration

    def value(name):
        return LaunchConfiguration(name).perform(context)

    if value("bag_file") == "b":
        return []
    namespace = resolve_namespace(
        context.launch_configurations.get("ros_namespace", ""), value("ouster_ns"))
    command, delay, profile_path, generated = prepare_replay(
        value("bag_file"), namespace, value("raw_packet_qos_depth"),
        value("raw_packet_qos_reliability"), value("play_rate"),
        value("play_delay"), value("loop"), value("qos_profile_overrides_path"))

    def cleanup(_context):
        cleanup_profile(profile_path, generated)
        return []

    player = ExecuteProcess(cmd=command, output="screen")
    return [
        RegisterEventHandler(OnShutdown(on_shutdown=[OpaqueFunction(function=cleanup)])),
        RegisterEventHandler(OnProcessExit(target_action=player,
                                          on_exit=[OpaqueFunction(function=cleanup)])),
        TimerAction(period=delay, actions=[player]),
    ]


def generate_launch_description():
    from launch import LaunchDescription
    from launch.actions import DeclareLaunchArgument, OpaqueFunction

    arguments = [DeclareLaunchArgument("bag_file")]
    for name, default in (
            ("ouster_ns", "ouster"), ("raw_packet_qos_depth", "1024"),
            ("raw_packet_qos_reliability", "reliable"), ("play_rate", "1.0"),
            ("play_delay", "0"), ("loop", "false"),
            ("qos_profile_overrides_path", "")):
        arguments.append(DeclareLaunchArgument(name, default_value=default))
    return LaunchDescription([*arguments, OpaqueFunction(function=launch_player)])
