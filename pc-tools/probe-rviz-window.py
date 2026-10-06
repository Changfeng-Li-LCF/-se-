"""Inspect RViz windows; --activate raises only explicitly requested process windows."""
import argparse
import ctypes
import ctypes.util
import json
import os
import re
import subprocess


def run(args):
    env = dict(os.environ, DISPLAY=':0')
    return subprocess.run(args, text=True, stdout=subprocess.PIPE,
                          stderr=subprocess.DEVNULL, timeout=3, env=env).stdout


def probe(pids=None):
    if pids is None:
        pids = [int(p) for p in run(['pgrep', '-x', 'rviz2']).split()]
    result = {'pids': pids, 'dialogs': [], 'windows': [], 'rendering_verified': False}
    if not pids:
        return result
    tree = run(['xwininfo', '-root', '-tree'])
    windows = re.findall(r'^\s*(0x[0-9a-f]+).*\("rviz2" "rviz2"\)', tree, re.M)
    for window in windows:
        properties = run(['xprop', '-id', window, '_NET_WM_PID',
                          '_NET_WM_WINDOW_TYPE', 'WM_TRANSIENT_FOR', '_NET_WM_NAME'])
        pid = re.search(r'_NET_WM_PID\(CARDINAL\) = (\d+)', properties)
        if not pid or int(pid[1]) not in pids:
            continue
        item = {'window': window, 'pid': int(pid[1])}
        title = re.search(r'_NET_WM_NAME\(UTF8_STRING\) = "(.*)"', properties)
        item['title'] = title[1] if title else ''
        if '_NET_WM_WINDOW_TYPE_DIALOG' in properties:
            info = run(['xwininfo', '-id', window])
            if 'Map State: IsViewable' in info:
                result['dialogs'].append(item)
        elif '_NET_WM_WINDOW_TYPE_NORMAL' in properties and item['title'].endswith(' - RViz'):
            info = run(['xwininfo', '-id', window])
            item['viewable'] = 'Map State: IsViewable' in info
            result['windows'].append(item)
    return result


def activate(window):
    """WSLg X11 raise/focus; it does not establish ROS or OpenGL rendering health."""
    lib = ctypes.CDLL(ctypes.util.find_library('X11'))
    lib.XOpenDisplay.argtypes = [ctypes.c_char_p]
    lib.XOpenDisplay.restype = ctypes.c_void_p
    lib.XMapRaised.argtypes = [ctypes.c_void_p, ctypes.c_ulong]
    lib.XSetInputFocus.argtypes = [ctypes.c_void_p, ctypes.c_ulong, ctypes.c_int, ctypes.c_ulong]
    lib.XSync.argtypes = [ctypes.c_void_p, ctypes.c_int]
    lib.XGetInputFocus.argtypes = [ctypes.c_void_p, ctypes.POINTER(ctypes.c_ulong), ctypes.POINTER(ctypes.c_int)]
    lib.XCloseDisplay.argtypes = [ctypes.c_void_p]
    display = lib.XOpenDisplay(b':0')
    if not display:
        raise RuntimeError('Cannot access the WSLg X11 display')
    try:
        ident = int(window, 16)
        lib.XMapRaised(display, ident)
        lib.XSync(display, 0)
        lib.XSetInputFocus(display, ident, 2, 0)
        lib.XSync(display, 0)
        focused, revert = ctypes.c_ulong(), ctypes.c_int()
        lib.XGetInputFocus(display, ctypes.byref(focused), ctypes.byref(revert))
        return {'activation_requested': True, 'x11_focus_confirmed': focused.value == ident,
                'focused_window': hex(focused.value)}
    finally:
        lib.XCloseDisplay(display)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--pid', type=int, action='append')
    parser.add_argument('--activate', action='store_true')
    args = parser.parse_args()
    snapshot = probe(args.pid)
    if args.activate:
        if not args.pid:
            raise SystemExit('--activate requires an explicit --pid')
        if snapshot['dialogs']:
            raise RuntimeError('The requested RViz window is waiting for a dialog')
        if snapshot['windows']:
            snapshot.update(activate(snapshot['windows'][0]['window']))
            snapshot['windows'] = probe(args.pid)['windows']
        else:
            snapshot.update(activation_requested=False, x11_focus_confirmed=False)
    print(json.dumps(snapshot))


if __name__ == '__main__':
    main()
