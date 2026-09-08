"""Regression tests for GitHub Action configuration, without Docker or ROS."""

import os
from pathlib import Path
import shutil
import subprocess
import tempfile
import unittest


class ActionConfigTests(unittest.TestCase):
    def run_action(self, config):
        source = Path(__file__).resolve().parents[1]
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            (root / ".github").mkdir()
            scripts = root / "industrial_ci/src"
            scripts.mkdir(parents=True)
            shutil.copy2(source / ".github/action.sh", root / ".github/action.sh")
            shutil.copy2(source / "industrial_ci/src/util.sh", scripts / "util.sh")
            shutil.copytree(source / "industrial_ci/src/folding", scripts / "folding")
            # Stop at the Docker boundary and record the environment forwarded by
            # the real action entrypoint and configuration parser.
            (scripts / "ci_main.sh").write_text(
                'printf "%s\\n" "${ROS_DISTRO-}" "${PYLINT_CHECK-}" '
                '"${BEFORE_SCRIPT-}" > "$ACTION_TEST_OUTPUT"\n'
            )
            output = root / "environment"
            env = dict(os.environ, GITHUB_WORKSPACE=directory,
                       GITHUB_REPOSITORY="example/robot", GITHUB_EVENT_NAME="push",
                       INPUT_CONFIG=config, ACTION_TEST_OUTPUT=str(output))
            for key in ("ROS_DISTRO", "PYLINT_CHECK", "BEFORE_SCRIPT", "ABICHECK_MERGE", "ACT"):
                env.pop(key, None)
            result = subprocess.run(["bash", str(root / ".github/action.sh")],
                                    env=env, text=True, capture_output=True)
            values = output.read_text().splitlines() if output.exists() else None
            return result, values

    def test_empty_and_filtered_configs_reach_ci(self):
        for config in ("", "{}", '{"label": "jazzy"}'):
            with self.subTest(config=config):
                result, values = self.run_action(config)
                self.assertEqual(result.returncode, 0, result.stderr)
                self.assertEqual(values, ["", "", ""])

    def test_supported_values_are_exported(self):
        result, values = self.run_action(
            '{"ROS_DISTRO":"jazzy","PYLINT_CHECK":true,"label":"ignored"}')
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertEqual(values, ["jazzy", "true", ""])

    def test_doubled_dollar_hook_is_preserved(self):
        result, values = self.run_action('{"BEFORE_SCRIPT":"echo $$ROS_DISTRO"}')
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertEqual(values, ["", "", "echo $ROS_DISTRO"])

    def test_invalid_json_does_not_reach_ci(self):
        result, values = self.run_action('{"ROS_DISTRO":')
        self.assertNotEqual(result.returncode, 0)
        self.assertIsNone(values)


if __name__ == "__main__":
    unittest.main()
