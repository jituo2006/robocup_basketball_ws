"""Validate generated launch argv without importing ROS or starting any node."""
import importlib.util
from pathlib import Path
import shutil
import subprocess
import sys
from types import ModuleType, SimpleNamespace as NS


def test_launch_passes_resolved_arguments_as_distinct_bash_parameters(monkeypatch):
    class Action:
        def __init__(self, *args, **kwargs):
            self.args = args
            self.__dict__.update(kwargs)
    class Configuration:
        def __init__(self, name): self.name = name

    modules = {
        'ament_index_python.packages': dict(get_package_share_directory=lambda pkg: '/install/share/'+pkg),
        'launch': dict(LaunchDescription=lambda entities: NS(entities=entities)),
        'launch.actions': dict(DeclareLaunchArgument=Action, ExecuteProcess=Action, TimerAction=Action),
        'launch.conditions': dict(IfCondition=Action),
        'launch.substitutions': dict(LaunchConfiguration=Configuration),
        'launch_ros.actions': dict(Node=Action),
    }
    for name, attrs in modules.items():
        mod = ModuleType(name)
        mod.__dict__.update(attrs)
        monkeypatch.setitem(sys.modules, name, mod)
    path = Path(__file__).parents[1]/'launch'/'bringup_lidar_vision.launch.py'
    spec = importlib.util.spec_from_file_location('_cold_boot_launch', path)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    actions = mod.generate_launch_description().entities
    defaults = {a.args[0]: a.default_value for a in actions if hasattr(a, 'default_value')}
    defaults.update(lidar_interface='radar-interface', lidar_host_ip='192.168.1.110', lidar_ptp_mode='on')
    session = next(a for a in actions if getattr(a, 'name', '') == 'lidar_session')
    command = [defaults[x.name] if isinstance(x, Configuration) else x for x in session.cmd]
    assert command[0:2] == ['bash', '-c']
    assert command[3:] == ['lidar_session', '--interface', 'radar-interface',
                            '--host-ip', '192.168.1.110', '--ptp-mode', 'on']
    assert command[2].endswith('"$@"')
    assert 'object at' not in command[2]
    assert session.additional_env['ROS_DOMAIN_ID'].name == 'ros_domain_id'
    # Real bash parameter forwarding, using only printf in place of all ROS commands.
    # A literal containing spaces also must remain one argument.
    bash = shutil.which('bash')
    assert bash
    args = command[3:]+['literal with spaces']
    result = subprocess.run([bash, '-c', 'printf "%s\\n" "$@"', *args],
                            capture_output=True, text=True, check=True)
    assert result.stdout.splitlines() == args[1:]
