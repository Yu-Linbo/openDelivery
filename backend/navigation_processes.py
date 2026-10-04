"""Stop one navigation stack, including children orphaned by a launcher exit."""
import os
from pathlib import Path
import signal
import time


NAVIGATION_BINARIES = {
    'controller_server', 'planner_server', 'recoveries_server', 'bt_navigator',
    'waypoint_follower', 'lifecycle_manager',
}


def navigation_processes(robot_id, domain_id, proc_root=Path('/proc')):
    """Match actual argv and ROS domain, never shell text or shared robot groups."""
    matches = {}
    namespace = '__ns:=/' + robot_id + '/navigation'
    for directory in Path(proc_root).glob('[0-9]*'):
        try:
            argv = [part.decode() for part in (directory / 'cmdline').read_bytes().split(b'\0') if part]
            env = dict(part.split(b'=', 1) for part in (directory / 'environ').read_bytes().split(b'\0')
                       if b'=' in part)
            if env.get(b'ROS_DOMAIN_ID', b'0').decode() != str(domain_id):
                continue
            if not argv:
                continue
            executable = Path(argv[0])
            node = namespace in argv and (
                executable.name in NAVIGATION_BINARIES and executable.parent.name.startswith('nav2_')
                or executable.name.startswith('python') and len(argv) > 1
                and Path(argv[1]).name == 'navigation_task_node'
                and Path(argv[1]).parent.name == 'navigation_tasks'
            )
            launcher = any(
                Path(arg).name == 'ros2' and argv[index+1:index+4] == ['launch', 'nav_bringup', 'stack.launch.py']
                and 'robot_name:=' + robot_id in argv[index+4:]
                for index, arg in enumerate(argv[:2])
            )
            if node or launcher:
                # starttime guards against PID reuse while waiting for exit.
                stat = (directory / 'stat').read_text().rsplit(')', 1)[1].split()
                if stat[0] != 'Z':
                    matches[int(directory.name)] = (stat[19], launcher)
        except (OSError, ValueError, IndexError, UnicodeError):
            continue
    return matches


def stop_navigation(robot_id, domain_id=None):
    domain = str(domain_id if domain_id is not None else os.environ.get('ROS_DOMAIN_ID', '0'))
    matches = navigation_processes(robot_id, domain)

    def survivors():
        current = navigation_processes(robot_id, domain)
        return {pid: identity for pid, identity in matches.items() if current.get(pid) == identity}

    def send(pids, sig):
        for pid in pids:
            try:
                os.kill(pid, sig)
            except ProcessLookupError:
                pass

    # Let launch forward SIGINT first. Orphaned nodes have no live launcher;
    # terminate them explicitly after the bounded graceful shutdown interval.
    send([pid for pid, (_, launcher) in matches.items() if launcher], signal.SIGINT)
    for _ in range(20):
        if not survivors():
            return sorted(matches)
        time.sleep(.1)
    send(survivors(), signal.SIGTERM)
    for _ in range(20):
        if not survivors():
            return sorted(matches)
        time.sleep(.1)
    send(survivors(), signal.SIGKILL)
    for _ in range(10):
        if not survivors():
            return sorted(matches)
        time.sleep(.1)
    remaining = survivors()
    if remaining:
        raise RuntimeError('navigation processes did not exit: ' + str(sorted(remaining)))
    return sorted(matches)
