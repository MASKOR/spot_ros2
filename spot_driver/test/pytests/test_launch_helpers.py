# Copyright (c) 2023-2024 Boston Dynamics AI Institute LLC. See LICENSE file for more info.

"""
Tests to check the ROS launch helper utilities.
"""

import tempfile
import unittest

import yaml
from launch import LaunchContext, Substitution
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution

from spot_driver.launch.spot_launch_helpers import get_name_and_prefix, substitute_launch_parameters


class LaunchHelpersTest(unittest.TestCase):
    def setUp(self) -> None:
        self.name_key: str = "spot_name"
        self.prefix_key: str = "frame_prefix"
        self.user_key: str = "username"

        self.name_value: str = "test_spot"
        self.prefix_value: str = "test_prefix_"
        self.user_value: str = "test_username"

        self.context = LaunchContext()

    def test_substitute_launch_parameters(self) -> None:
        """
        Test the util substitute_launch_parameters.
        """
        with tempfile.NamedTemporaryFile(mode="w", suffix="config.yaml") as temp:
            data = {
                self.name_key: self.name_value,
                self.prefix_key: self.prefix_value,
                self.user_key: self.user_value,
            }
            yaml.dump({"/**": {"ros__parameters": data}}, temp.file)
            temp.file.close()

            # Empty substitutions should not modify the yaml file params.
            params = substitute_launch_parameters(temp.file.name, {}, self.context)
            self.assertEqual(params[self.name_key], self.name_value, "Substitution empty, should not change.")
            self.assertEqual(params[self.prefix_key], self.prefix_value, "Substitution empty, should not change.")
            self.assertEqual(params[self.user_key], self.user_value, "Substitution empty, should not change.")

            # Empty launch arguments should not modify the yaml file params.
            substitutions = {
                self.name_key: LaunchConfiguration(self.name_key, default=""),
                self.prefix_key: LaunchConfiguration(self.prefix_key, default=""),
                self.user_key: LaunchConfiguration(self.user_key, default=""),
            }
            params = substitute_launch_parameters(temp.file.name, substitutions, self.context)
            self.assertEqual(params[self.name_key], self.name_value, "Launch argument empty, should not change.")
            self.assertEqual(params[self.prefix_key], self.prefix_value, "Launch argument empty, should not change.")
            self.assertEqual(params[self.user_key], self.user_value, "Launch argument empty, should not change.")

            # Substitutions should modify only their corresponding keys.
            substitutions = {
                self.name_key: LaunchConfiguration(self.name_key, default="spot_name_overridden"),
                self.user_key: LaunchConfiguration(self.user_key, default="username_overridden"),
            }
            params = substitute_launch_parameters(temp.file.name, substitutions, self.context)
            self.assertIsInstance(params[self.name_key], Substitution, "Launch argument set, should override.")
            self.assertEqual(
                params[self.name_key].perform(self.context),
                "spot_name_overridden",
                "Launch argument set, should override.",
            )
            self.assertEqual(params[self.prefix_key], self.prefix_value, "Substitution empty, should not change.")
            self.assertIsInstance(params[self.user_key], Substitution, "Launch argument set, should override.")
            self.assertEqual(
                params[self.user_key].perform(self.context),
                "username_overridden",
                "Launch argument set, should override.",
            )
            substitutions = {
                self.prefix_key: PathJoinSubstitution(["prefix_overridden", ""]),
            }
            params = substitute_launch_parameters(temp.file.name, substitutions, self.context)
            self.assertEqual(params[self.name_key], self.name_value, "Substitution empty, should not change.")
            self.assertIsInstance(params[self.prefix_key], Substitution, "Substitution set, should override.")
            self.assertEqual(
                params[self.prefix_key].perform(self.context),
                "prefix_overridden/",
                "Substitution set, should override.",
            )
            self.assertEqual(params[self.user_key], self.user_value, "Substitution empty, should not change.")

            # Giving non-substitution types as parameter substitutions should fail.
            substitutions = {self.name_key: "overridden"}
            self.assertRaises(
                AttributeError,
                substitute_launch_parameters,
                temp.file.name,
                substitutions,
                self.context,
            )

    def test_get_name_and_prefix(self) -> None:
        """
        Test the util get_name_and_prefix.
        """
        name, prefix = get_name_and_prefix({})
        self.assertTrue(name == "" and prefix == "", "Empty parameters.")

        name, prefix = get_name_and_prefix({self.name_key: ""})
        self.assertTrue(name == "" and prefix == "", "Empty parameters.")

        name, prefix = get_name_and_prefix({self.name_key: "", self.prefix_key: ""})
        self.assertTrue(name == "" and prefix == "", "Empty parameters.")

        name, prefix = get_name_and_prefix({self.name_key: self.name_value})
        self.assertTrue(name == self.name_value and prefix == self.name_value + "/", "Prefix from name.")

        name, prefix = get_name_and_prefix({self.name_key: self.name_value, self.prefix_key: ""})
        self.assertTrue(name == self.name_value and prefix == "", "Explicit prefix.")

        name, prefix = get_name_and_prefix({self.name_key: self.name_value, self.prefix_key: self.prefix_value})
        self.assertTrue(name == self.name_value and prefix == self.prefix_value, "Explicit prefix.")

        name, prefix = get_name_and_prefix({self.name_key: "", self.prefix_key: self.prefix_value})
        self.assertTrue(name == "" and prefix == self.prefix_value, "Explicit prefix.")

        # Should also work when values are Substitution types.
        name_launch_param = LaunchConfiguration(self.name_key, default=self.name_value)
        prefix_launch_param = LaunchConfiguration(self.prefix_key, default=self.prefix_value)

        name, prefix = get_name_and_prefix({self.name_key: name_launch_param})
        self.assertTrue(
            isinstance(name, Substitution) and isinstance(prefix, Substitution), "Launch argument: prefix from name."
        )
        self.assertTrue(
            name.perform(self.context) == self.name_value and prefix.perform(self.context) == self.name_value + "/",
            "Launch argument: prefix from name.",
        )

        name, prefix = get_name_and_prefix({self.name_key: name_launch_param, self.prefix_key: prefix_launch_param})
        self.assertTrue(
            isinstance(name, Substitution) and isinstance(prefix, Substitution), "Launch argument: explicit prefix."
        )
        self.assertTrue(
            name.perform(self.context) == self.name_value and prefix.perform(self.context) == self.prefix_value,
            "Launch argument: explicit prefix.",
        )

        name_path_join_substitution = PathJoinSubstitution(self.name_value)
        prefix_path_join_substitution = PathJoinSubstitution([self.prefix_value, ""])

        name, prefix = get_name_and_prefix({self.name_key: name_path_join_substitution})
        self.assertTrue(
            isinstance(name, Substitution) and isinstance(prefix, Substitution), "Substitution: prefix from name."
        )
        self.assertTrue(
            name.perform(self.context) == self.name_value and prefix.perform(self.context) == self.name_value + "/",
            "Substitution: prefix from name.",
        )

        name, prefix = get_name_and_prefix(
            {self.name_key: name_path_join_substitution, self.prefix_key: prefix_path_join_substitution}
        )
        self.assertTrue(
            isinstance(name, Substitution) and isinstance(prefix, Substitution), "Substitution: explicit prefix."
        )
        self.assertTrue(
            name.perform(self.context) == self.name_value and prefix.perform(self.context) == self.prefix_value + "/",
            "Substitution: explicit prefix.",
        )


if __name__ == "__main__":
    unittest.main()
