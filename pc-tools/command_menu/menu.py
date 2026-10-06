"""Desktop command picker. Opening it does not connect or start the car."""
import argparse
import json
from pathlib import Path
import queue
import sys
import threading
import traceback
import tkinter as tk
from tkinter import ttk, messagebox

import backend
from catalog import ACTIONS, BY_KEY


class CommandMenu:
    def __init__(self, root):
        self.root = root
        self.busy = False
        self.events = queue.Queue()
        root.title('小车常用命令 · MobaXterm')
        root.geometry('1060x720')
        root.minsize(880, 640)
        root.configure(bg='#f3f6fa')
        style = ttk.Style(root)
        style.theme_use('clam')
        style.configure('.', font=('Microsoft YaHei UI', 10))
        style.configure('Treeview', rowheight=33, font=('Microsoft YaHei UI', 11))
        style.configure('Treeview.Heading', font=('Microsoft YaHei UI', 10, 'bold'))
        style.configure('Run.TButton', font=('Microsoft YaHei UI', 11, 'bold'), padding=(15, 11))
        outer = ttk.Frame(root, padding=20)
        outer.pack(fill='both', expand=True)
        ttk.Label(outer, text='小车常用命令', font=('Microsoft YaHei UI', 21, 'bold')).pack(anchor='w')
        ttk.Label(outer, text='选择操作 → 发送并执行 → 在 MobaXterm 新标签页查看输出', foreground='#516175').pack(anchor='w', pady=(5, 10))
        self.connection = tk.StringVar()
        self.update_connection_label()
        top = ttk.Frame(outer)
        top.pack(fill='x', pady=(0, 12))
        ttk.Label(top, textvariable=self.connection).pack(side='left')
        self.check_button = ttk.Button(top, text='刷新车端连接', command=lambda: self.background(None))
        self.check_button.pack(side='right')
        search_row = ttk.Frame(outer)
        search_row.pack(fill='x', pady=(0, 10))
        ttk.Label(search_row, text='查找命令  ').pack(side='left')
        self.search = tk.StringVar()
        entry = ttk.Entry(search_row, textvariable=self.search)
        entry.pack(fill='x', expand=True)
        self.search.trace_add('write', lambda *_: self.populate())
        panes = ttk.Panedwindow(outer, orient='horizontal')
        panes.pack(fill='both', expand=True)
        left = ttk.Frame(panes)
        right = ttk.Frame(panes, padding=(18, 0, 0, 0))
        panes.add(left, weight=2)
        panes.add(right, weight=3)
        self.tree = ttk.Treeview(left, show='tree', selectmode='browse')
        scroll = ttk.Scrollbar(left, orient='vertical', command=self.tree.yview)
        self.tree.configure(yscrollcommand=scroll.set)
        scroll.pack(side='right', fill='y')
        self.tree.pack(fill='both', expand=True)
        self.tree.bind('<<TreeviewSelect>>', self.select)
        self.title = tk.StringVar(value='选择左侧命令')
        ttk.Label(right, textvariable=self.title, font=('Microsoft YaHei UI', 14, 'bold'), wraplength=500).pack(anchor='w')
        self.description = tk.StringVar()
        ttk.Label(right, textvariable=self.description, wraplength=470, justify='left', foreground='#405168').pack(anchor='w', pady=(12, 20))
        ttk.Label(right, text='操作内容', font=('Microsoft YaHei UI', 10, 'bold')).pack(anchor='w')
        self.preview = tk.Text(right, height=8, wrap='word', font=('Consolas', 10), bg='#edf2f8', relief='flat', padx=12, pady=12)
        self.preview.pack(fill='both', expand=True, pady=(8, 12))
        self.run_button = ttk.Button(right, text='发送并执行', style='Run.TButton', command=self.execute)
        self.run_button.pack(fill='x')
        self.copy_button = ttk.Button(right, text='复制车端命令', command=self.copy_command)
        self.copy_button.pack(fill='x', pady=(8, 0))
        ttk.Label(right, text='Ctrl+C 在执行命令的标签页中使用。\n菜单打开时不会自动发车。', foreground='#65758a').pack(anchor='w', pady=(12, 0))
        self.status = tk.StringVar(value='就绪。首次执行时会自动安装菜单所需的小工具。')
        ttk.Separator(outer).pack(fill='x', pady=(14, 10))
        ttk.Label(outer, textvariable=self.status, wraplength=980).pack(anchor='w')
        self.populate()
        self.root.after(100, self.poll)

    def update_connection_label(self, config=None):
        try:
            config = backend.read_connection() if config is None else config
            self.connection.set(f"车端：{config['username']}@{config['host']}:{config.get('port',22)}  ·  使用统一连接配置")
        except Exception as exc:
            self.connection.set('连接配置读取失败：' + str(exc))

    def populate(self):
        old = self.tree.selection()
        self.tree.delete(*self.tree.get_children())
        needle = self.search.get().strip().lower()
        groups = {}
        keys = []
        for action in ACTIONS:
            if needle and needle not in (action.title + action.group + action.description + action.key).lower():
                continue
            if action.group not in groups:
                groups[action.group] = self.tree.insert('', 'end', text=action.group, open=True)
            self.tree.insert(groups[action.group], 'end', iid=action.key, text='  ' + action.title)
            keys.append(action.key)
        if keys:
            self.tree.selection_set(old[0] if old and old[0] in keys else keys[0])
        self.select()

    def chosen(self):
        selected = self.tree.selection()
        return BY_KEY.get(selected[0]) if selected else None

    def select(self, _event=None):
        action = self.chosen()
        self.title.set(action.title if action else '选择左侧命令')
        self.description.set(action.description if action else '')
        self.preview.configure(state='normal')
        self.preview.delete('1.0', 'end')
        if action:
            self.preview.insert('1.0', action.example)
        self.preview.configure(state='disabled')
        self.controls()

    def controls(self):
        enabled = not self.busy and self.chosen() is not None
        self.run_button.configure(state='normal' if enabled else 'disabled')
        self.copy_button.configure(state='normal' if enabled else 'disabled')
        self.check_button.configure(state='disabled' if self.busy else 'normal')

    def execute(self):
        action = self.chosen()
        if action and not self.busy:
            self.background(action.key)

    def background(self, key):
        if self.busy:
            return
        self.busy = True
        self.controls()
        self.status.set('正在查找并验证车端连接…')

        def work():
            try:
                if key is not None:
                    backend.moba_path()  # Fail before connecting if the terminal is missing.
                config = backend.resolve_connection()
                self.events.put(('connection', config))
                if key is None:
                    self.events.put(('done', '车端连接已更新。没有执行车辆命令。'))
                    return
                self.events.put(('status', '正在准备命令工具…'))
                version = backend.install_payload(config)
                backend.launch_action(config, key, version)
                self.events.put(('done', '已发送到 MobaXterm 新标签页：' + BY_KEY[key].title + '。执行结果请看该终端。'))
            except Exception as exc:
                log = backend.ROOT / 'menu-error.log'
                with log.open('a', encoding='utf-8') as f:
                    traceback.print_exc(file=f)
                self.events.put(('error', str(exc)))

        threading.Thread(target=work, daemon=True).start()

    def poll(self):
        try:
            while True:
                kind, value = self.events.get_nowait()
                if kind == 'connection':
                    self.update_connection_label(value)
                else:
                    self.status.set(value)
                if kind in ('done', 'error'):
                    self.busy = False
                    self.controls()
                if kind == 'error':
                    messagebox.showerror('命令未发送', value, parent=self.root)
        except queue.Empty:
            pass
        self.root.after(100, self.poll)

    def copy_command(self):
        action = self.chosen()
        if action:
            self.root.clipboard_clear()
            self.root.clipboard_append(backend.remote_command(action.key))
            self.status.set('已复制车端命令。若尚未安装命令工具，请先使用“发送并执行”。')


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--smoke-test', action='store_true')
    parser.add_argument('--install-only', action='store_true')
    args = parser.parse_args()
    if args.install_only:
        config = backend.resolve_connection()
        print(json.dumps({'host': config['host'], 'version': backend.install_payload(config), 'vehicle_commands_sent': False}))
        return
    root = tk.Tk()
    if args.smoke_test:
        root.withdraw()
    app = CommandMenu(root)
    if args.smoke_test:
        root.update()
        assert sum(len(app.tree.get_children(g)) for g in app.tree.get_children()) == len(ACTIONS)
        app.search.set('地图')
        root.update()
        assert app.tree.exists('save_map') and app.tree.exists('view_map')
        assert not app.tree.exists('figure8')
        root.destroy()
        print('PASS: GUI renders all commands and filters; no connection or vehicle command')
    else:
        root.mainloop()


if __name__ == '__main__':
    main()
