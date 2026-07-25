from pathlib import Path

from ament_index_python.packages import get_package_prefix, get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, SetEnvironmentVariable
from launch.substitutions import EnvironmentVariable, LaunchConfiguration


def generate_launch_description():
    share = Path(get_package_share_directory("livox_laser_simulation"))
    prefix = Path(get_package_prefix("livox_laser_simulation"))
    models = share / "models"
    plugins = prefix / "lib"

    return LaunchDescription([
        SetEnvironmentVariable(
            "GZ_SIM_RESOURCE_PATH",
            [str(models), ":", ":",
             EnvironmentVariable("GZ_SIM_RESOURCE_PATH", default_value="")]),
        SetEnvironmentVariable(
            "GZ_SIM_SYSTEM_PLUGIN_PATH",
            f"{plugins}:{__import__('os').environ.get('GZ_SIM_SYSTEM_PLUGIN_PATH', '')}"),
        ExecuteProcess(cmd=["gz", "sim", "-r", str(share / "worlds" / "mid360_gz.sdf")], output="screen"),
    ])
