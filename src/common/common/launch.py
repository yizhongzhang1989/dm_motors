"""Launch substitutions reading the same configuration as runtime nodes."""

from launch import Substitution
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

from .config import ConfigManager, default_config_path


def config_argument():
    return DeclareLaunchArgument("config_file", default_value=default_config_path(),
                                 description="Single package/launch runtime YAML configuration.")


def config_for_context(context):
    return ConfigManager(LaunchConfiguration("config_file", default=default_config_path()).perform(context))


class ConfigValue(Substitution):
    def __init__(self, key, package, filename, node=None):
        super().__init__()
        self.key = key
        self.package = package
        self.filename = filename
        self.node = node

    def perform(self, context):
        profile = profile_for_context(context, self.package, self.filename)
        value = config_for_context(context).launch_value(*profile, self.key, node=self.node)
        if isinstance(value, bool):
            return str(value).lower()
        if type(value) not in (str, int, float):
            raise ValueError(f"{self.key} must be a scalar launch argument")
        return str(value)

    def describe(self):
        return f"configuration[{self.package}/{self.filename}:{self.key}]"


def profile_arguments(package, filename):
    return [
        DeclareLaunchArgument("config_package", default_value=package,
                              description="Package owning the selected launch configuration profile."),
        DeclareLaunchArgument("config_launch", default_value=filename,
                              description="Full launch filename owning the selected configuration profile."),
    ]


def profile_for_context(context, package, filename):
    return (
        LaunchConfiguration("config_package", default=package).perform(context),
        LaunchConfiguration("config_launch", default=filename).perform(context),
    )
