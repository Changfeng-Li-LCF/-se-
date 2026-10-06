"""Raise the Windows WSLg window corresponding to a verified Linux RViz title.

X11 input focus alone does not bring the RemoteApp host window to the desktop
foreground. This module never presses a key or dismisses a user dialog.
"""
import argparse
import ctypes
from ctypes import wintypes as w
import json
import os


class WindowsWindowNotReady(RuntimeError):
    pass


def select_window(windows, titles):
    expected = {title+' (RacecarUbuntu2204)' for title in titles}
    matches = [item for item in windows if item['class'] == 'RAIL_WINDOW'
               and any(item['title'] == value or item['title'].endswith('] '+value)
                       for value in expected)]
    if not matches:
        raise WindowsWindowNotReady('The matching WSLg Windows window has not appeared yet')
    if len(matches) != 1:
        raise RuntimeError('Expected one matching Windows RViz window; found '+str(len(matches)))
    return matches[0]


class Windows:
    def __init__(self):
        if os.name != 'nt':
            raise RuntimeError('Windows foreground activation must run on the Windows host')
        self.user = ctypes.WinDLL('user32', use_last_error=True)
        self.kernel = ctypes.WinDLL('kernel32', use_last_error=True)
        self.callback = ctypes.WINFUNCTYPE(w.BOOL, w.HWND, w.LPARAM)
        declarations = {
            'EnumWindows': ([self.callback, w.LPARAM], w.BOOL),
            'GetWindowTextW': ([w.HWND, w.LPWSTR, ctypes.c_int], ctypes.c_int),
            'GetClassNameW': ([w.HWND, w.LPWSTR, ctypes.c_int], ctypes.c_int),
            'GetWindowThreadProcessId': ([w.HWND, ctypes.POINTER(w.DWORD)], w.DWORD),
            'GetForegroundWindow': ([], w.HWND),
            'GetLastActivePopup': ([w.HWND], w.HWND),
            'GetWindow': ([w.HWND, w.UINT], w.HWND),
            'IsWindowVisible': ([w.HWND], w.BOOL),
            'IsIconic': ([w.HWND], w.BOOL),
            'ShowWindow': ([w.HWND, ctypes.c_int], w.BOOL),
            'BringWindowToTop': ([w.HWND], w.BOOL),
            'SetForegroundWindow': ([w.HWND], w.BOOL),
            'AttachThreadInput': ([w.DWORD, w.DWORD, w.BOOL], w.BOOL),
        }
        for name, (args, result) in declarations.items():
            api = getattr(self.user, name)
            api.argtypes, api.restype = args, result
        self.kernel.GetCurrentThreadId.argtypes = []
        self.kernel.GetCurrentThreadId.restype = w.DWORD

    def info(self, handle):
        title, cls, pid = ctypes.create_unicode_buffer(2048), ctypes.create_unicode_buffer(256), w.DWORD()
        self.user.GetWindowTextW(handle, title, len(title))
        self.user.GetClassNameW(handle, cls, len(cls))
        self.user.GetWindowThreadProcessId(handle, ctypes.byref(pid))
        return {'handle': handle, 'title': title.value, 'class': cls.value,
                'pid': pid.value, 'visible': bool(self.user.IsWindowVisible(handle))}

    def windows(self):
        values = []
        @self.callback
        def collect(handle, _):
            values.append(self.info(handle))
            return True
        self.user.EnumWindows(collect, 0)
        return values

    def focus(self, main):
        user = self.user
        # Recheck identity immediately before any window mutation.
        fresh = self.info(main['handle'])
        if any(fresh[key] != main[key] for key in ('title', 'pid', 'class')):
            raise RuntimeError('Windows RViz identity changed before activation')
        target = main['handle']
        popup = user.GetLastActivePopup(target)
        if popup and popup != target and user.IsWindowVisible(popup) and user.GetWindow(popup, 4) == target:
            target = popup
        current = self.kernel.GetCurrentThreadId()
        pid = w.DWORD()
        foreground_thread = user.GetWindowThreadProcessId(user.GetForegroundWindow(), ctypes.byref(pid))
        attached = False
        try:
            if foreground_thread and foreground_thread != current:
                attached = bool(user.AttachThreadInput(current, foreground_thread, True))
            user.ShowWindow(target, 9 if user.IsIconic(target) else 5)
            user.BringWindowToTop(target)
            user.SetForegroundWindow(target)
            confirmed = user.GetForegroundWindow() == target
            return {'windows_focus_confirmed': confirmed,
                    'windows_foreground_handle': target if confirmed else None,
                    'windows_target_role': 'main' if target == main['handle'] else 'owned_popup',
                    'windows_main_handle': main['handle']}
        finally:
            if attached:
                user.AttachThreadInput(current, foreground_thread, False)


def focus_titles(titles, backend=None):
    backend = backend or Windows()
    main = select_window(backend.windows(), titles)
    return backend.focus(main)


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--title', action='append', required=True)
    args = parser.parse_args()
    print(json.dumps(focus_titles(args.title), ensure_ascii=False))
