"""Startup dependency and reuse policy; no ROS imports or motion side effects."""
from pathlib import Path


def add_shutdown_options(parser):
    group = parser.add_mutually_exclusive_group()
    parser.set_defaults(shutdown_after_run=False)
    group.add_argument('--shutdown-after-run', action='store_true',
                       help='Stop the owned stack after this run; saved-map runs otherwise keep it stopped and ready')
    group.add_argument('--keep-stack', action='store_false', dest='shutdown_after_run',
                       help='Keep a ready saved-map/FollowPath stack for the next command (default)')


def restart_reason(record, fingerprint, *, follow_path_only, saved_map=None,
                   fresh_mapping=False, explicit=False):
    if explicit:
        return 'explicit restart requested'
    if fresh_mapping:
        return 'new SLAM map requested'
    if record.get('startup_schema') != 2:
        return 'startup implementation changed'
    if not record.get('idle_verified', False):
        return 'previous session did not confirm a stopped reusable stack'
    if record.get('fingerprint') != fingerprint:
        return 'configuration, installed code or saved-map contents changed'
    requested_map = str(Path(saved_map).expanduser().resolve()) if saved_map else None
    previous_map = record.get('saved_map')
    if previous_map:
        previous_map = str(Path(previous_map).expanduser().resolve())
    if previous_map != requested_map:
        return 'saved map or localization mode changed'
    if record.get('follow_path_only', True) != follow_path_only:
        return 'navigation mode changed'
    return None


def prepare_saved_map(*, reused, stop_previous, wait_sensors,
                      start_localization, localize, start_navigation):
    # A retained process must not retain permission to drive during relocalization.
    if reused:
        stop_previous()
    wait_sensors()
    if not reused:
        start_localization()
    localize()
    if not reused:
        start_navigation()


def cancellation_actions(record):
    # Use the currently owned stack, not the mode requested for its replacement.
    if record.get('saved_map'):
        return ('/navigate_through_poses', '/navigate_to_pose', '/follow_path')
    if not record.get('follow_path_only', True):
        return ('/navigate_to_pose', '/follow_path')
    return ('/follow_path',)
