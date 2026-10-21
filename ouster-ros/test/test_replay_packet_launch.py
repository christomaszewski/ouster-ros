"""No ROS installation or sensor is needed for these launch-plan checks."""

import importlib.util
import json
from pathlib import Path
import sys
import tempfile
import unittest


SOURCE = Path(__file__).resolve().parents[1] / "launch/replay_packets.launch.py"
sys.dont_write_bytecode = True
SPEC = importlib.util.spec_from_file_location("ouster_replay_packets_launch", SOURCE)
REPLAY = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(REPLAY)


class ReplayPacketLaunchTest(unittest.TestCase):
    def prepare(self, **overrides):
        arguments = dict(bag_file="/data/bag with spaces", namespace="ouster",
                         depth="1024", reliability="reliable", rate="1.0",
                         delay="0", loop="false", override_path="")
        arguments.update(overrides)
        result = REPLAY.prepare_replay(**arguments)
        self.addCleanup(REPLAY.cleanup_profile, result[2], result[3])
        return result

    def test_default_profile_upgrades_legacy_bag_offers(self):
        command, delay, path, generated = self.prepare()
        self.assertTrue(generated)
        self.assertEqual(delay, 0)
        self.assertEqual(command[:4], ["ros2", "bag", "play", "/data/bag with spaces"])
        self.assertIn("--clock", command)
        self.assertEqual(command[command.index("--qos-profile-overrides-path") + 1], str(path))
        profiles = json.loads(path.read_text())
        for namespace in ("/os_node", "/ouster"):
            self.assertEqual(profiles[namespace + "/metadata"], {
                "history": "keep_last", "depth": 1, "reliability": "reliable",
                "durability": "transient_local"})
            for topic in ("lidar_packets", "imu_packets"):
                self.assertEqual(profiles[namespace + "/" + topic], {
                    "history": "keep_last", "depth": 1024,
                    "reliability": "reliable", "durability": "volatile"})
                self.assertIn(namespace + "/" + topic, profiles)
        self.assertEqual(command[command.index("--remap") + 1:], [
            "/os_node/metadata:=/ouster/metadata",
            "/os_node/lidar_packets:=/ouster/lidar_packets",
            "/os_node/imu_packets:=/ouster/imu_packets"])

    def test_custom_namespace_remaps_both_recording_aliases(self):
        command, _, path, _ = self.prepare(namespace="/rig/top/")
        profiles = json.loads(path.read_text())
        for topic in ("metadata", "lidar_packets", "imu_packets"):
            self.assertIn("/rig/top/" + topic, profiles)
            for original in ("/os_node", "/ouster"):
                self.assertIn(original + "/" + topic + ":=/rig/top/" + topic, command)

    def test_root_and_parent_namespace_resolution(self):
        self.assertEqual(REPLAY.resolve_namespace("/rig", "top"), "rig/top")
        self.assertEqual(REPLAY.resolve_namespace("/rig", "/top"), "/top")
        self.assertEqual(REPLAY.resolve_namespace("/rig", "/"), "/")
        command, _, path, _ = self.prepare(namespace="/")
        self.assertIn("/metadata", json.loads(path.read_text()))
        self.assertIn("/ouster/lidar_packets:=/lidar_packets", command)

    def test_options_and_custom_override_are_preserved(self):
        with tempfile.TemporaryDirectory() as directory:
            override = Path(directory) / "custom override.yaml"
            contents = "/rig/top/lidar_packets:\n  reliability: best_effort\n"
            override.write_text(contents)
            command, delay, path, generated = self.prepare(
                namespace="rig/top", depth="4096", reliability="best_effort",
                rate="2.5", delay="0.25", loop="true", override_path=str(override))
            self.assertFalse(generated)
            self.assertEqual(path, override.resolve())
            self.assertEqual(delay, 0.25)
            self.assertEqual(command[command.index("--rate") + 1], "2.5")
            self.assertIn("--loop", command)
            REPLAY.cleanup_profile(path, generated)
            self.assertEqual(override.read_text(), contents)

    def test_best_effort_and_depth_reach_generated_profile(self):
        _, _, path, _ = self.prepare(depth="4096", reliability="best_effort")
        profile = json.loads(path.read_text())["/ouster/lidar_packets"]
        self.assertEqual(profile["depth"], 4096)
        self.assertEqual(profile["reliability"], "best_effort")

    def test_generated_profile_cleanup_is_idempotent(self):
        _, _, path, generated = self.prepare()
        REPLAY.cleanup_profile(path, generated)
        REPLAY.cleanup_profile(path, generated)
        self.assertFalse(path.exists())

    def test_invalid_argument_values_fail(self):
        for arguments in (
                {"depth": "0"}, {"depth": "-1"}, {"depth": "65537"},
                {"depth": "1.5"}, {"reliability": "automatic"},
                {"rate": "0"}, {"rate": "nan"}, {"rate": "inf"},
                {"delay": "-1"}, {"delay": "nan"}, {"loop": "perhaps"},
                {"namespace": "rig//top"}, {"namespace": "rig with spaces"}):
            with self.subTest(arguments=arguments), self.assertRaises(ValueError):
                self.prepare(**arguments)

    def test_explicit_override_must_be_a_readable_file(self):
        with tempfile.TemporaryDirectory() as directory:
            with self.assertRaises(FileNotFoundError):
                self.prepare(override_path=str(Path(directory) / "missing.yaml"))
            with self.assertRaises(IsADirectoryError):
                self.prepare(override_path=directory)


if __name__ == "__main__":
    unittest.main()
