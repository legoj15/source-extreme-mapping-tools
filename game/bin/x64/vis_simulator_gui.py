#!/usr/bin/env python3
"""
vis_simulator_gui — GUI wrapper for vis_simulator.py.
"""
from __future__ import annotations

import json
import os
import queue
import signal
import subprocess
import sys
import threading
import time
import tkinter as tk
from tkinter import filedialog, ttk
from pathlib import Path
from typing import Optional

_THIS_DIR = Path(__file__).resolve().parent
_SETTINGS_FILE = _THIS_DIR / '.vis_simulator_gui.json'
_SCRIPT_PATH = _THIS_DIR / 'vis_simulator.py'


def _load_settings() -> dict:
    try:
        return json.loads(_SETTINGS_FILE.read_text(encoding='utf-8'))
    except Exception:
        return {}


def _save_settings(data: dict) -> None:
    try:
        _SETTINGS_FILE.write_text(json.dumps(data, indent=2), encoding='utf-8')
    except Exception:
        pass


class ToolTip:
    def __init__(self, widget: tk.Widget, text: str, delay: int = 400):
        self.widget = widget
        self.text = text
        self.delay = delay
        self._tip_window = None
        self._after_id = None
        widget.bind('<Enter>', self._schedule, add='+')
        widget.bind('<Leave>', self._cancel, add='+')
        widget.bind('<ButtonPress>', self._cancel, add='+')

    def _schedule(self, _event=None):
        self._cancel()
        self._after_id = self.widget.after(self.delay, self._show)

    def _cancel(self, _event=None):
        if self._after_id:
            self.widget.after_cancel(self._after_id)
            self._after_id = None
        self._hide()

    def _show(self):
        if self._tip_window:
            return
        x = self.widget.winfo_rootx() + 20
        y = self.widget.winfo_rooty() + self.widget.winfo_height() + 4
        tw = tk.Toplevel(self.widget)
        tw.wm_overrideredirect(True)
        tw.wm_geometry(f'+{x}+{y}')
        tw.attributes('-topmost', True)
        label = tk.Label(
            tw, text=self.text, justify='left',
            background='#ffffe0', foreground='#1a1a2e',
            relief='solid', borderwidth=1,
            font=('Segoe UI', 9), wraplength=380,
            padx=6, pady=4,
        )
        label.pack()
        self._tip_window = tw

    def _hide(self):
        if self._tip_window:
            self._tip_window.destroy()
            self._tip_window = None


def _find_exe(name: str) -> str:
    p = _THIS_DIR / name
    return str(p) if p.exists() else ''


# Theming
_BG = '#1a1a2e'
_BG2 = '#16213e'
_BG3 = '#0f3460'
_FG = '#e0e0e0'
_FG_DIM = '#8899aa'
_ACCENT = '#e67e22'  # Orange accent for visibility sim
_ACCENT2 = '#d35400'
_GREEN = '#2ecc71'
_RED = '#e74c3c'
_YELLOW = '#f39c12'
_ENTRY_BG = '#22243a'
_ENTRY_FG = '#e0e0e0'
_BTN_BG = '#0f3460'
_BTN_FG = '#e0e0e0'

def _apply_theme(root: tk.Tk):
    style = ttk.Style(root)
    style.theme_use('clam')
    style.configure('.', background=_BG, foreground=_FG, fieldbackground=_ENTRY_BG, borderwidth=0, font=('Segoe UI', 10))
    style.configure('TFrame', background=_BG)
    style.configure('TLabel', background=_BG, foreground=_FG, font=('Segoe UI', 10))
    style.configure('TLabelframe', background=_BG, foreground=_ACCENT, font=('Segoe UI', 10, 'bold'))
    style.configure('TLabelframe.Label', background=_BG, foreground=_ACCENT, font=('Segoe UI', 10, 'bold'))
    style.configure('TEntry', fieldbackground=_ENTRY_BG, foreground=_ENTRY_FG, insertcolor=_ACCENT, font=('Segoe UI', 10))
    style.configure('TButton', background=_BTN_BG, foreground=_BTN_FG, font=('Segoe UI', 10, 'bold'), padding=(10, 4))
    style.map('TButton', background=[('active', _ACCENT2), ('pressed', _ACCENT)], foreground=[('active', '#ffffff')])
    style.configure('TCheckbutton', background=_BG, foreground=_FG, font=('Segoe UI', 10))
    style.map('TCheckbutton', background=[('active', _BG2)], foreground=[('active', _FG)])
    style.configure('TSpinbox', fieldbackground=_ENTRY_BG, foreground=_ENTRY_FG, arrowcolor=_ACCENT, font=('Segoe UI', 10))

    style.configure('Run.TButton', background=_ACCENT2, foreground='#ffffff', font=('Segoe UI', 11, 'bold'), padding=(16, 6))
    style.map('Run.TButton', background=[('active', _ACCENT), ('pressed', _GREEN)], foreground=[('active', '#ffffff')])
    style.configure('Cancel.TButton', background=_RED, foreground='#ffffff', font=('Segoe UI', 11, 'bold'), padding=(16, 6))
    style.map('Cancel.TButton', background=[('active', '#c0392b'), ('pressed', '#a93226')])
    style.configure('TProgressbar', troughcolor=_BG2, background=_ACCENT, thickness=6)
    style.configure('Status.TLabel', background=_BG2, foreground=_FG_DIM, font=('Segoe UI', 9))
    style.configure('StatusOk.TLabel', background=_BG2, foreground=_GREEN, font=('Segoe UI', 9, 'bold'))
    style.configure('StatusErr.TLabel', background=_BG2, foreground=_RED, font=('Segoe UI', 9, 'bold'))


class VisSimulatorGUI:
    def __init__(self):
        self.root = tk.Tk()
        self.root.title('Visibility Simulator Launcher')
        self.root.geometry('720x640')
        self.root.minsize(600, 500)
        self.root.configure(bg=_BG)
        _apply_theme(self.root)

        self._process = None
        self._reader_thread = None
        self._output_queue = queue.Queue()
        self._running = False
        self._start_time = 0.0
        self._settings = _load_settings()

        self._build_ui()
        self._restore_settings()
        self._auto_detect()

        self.root.after(100, self._poll_output)
        self.root.protocol('WM_DELETE_WINDOW', self._on_close)

    def run(self):
        self.root.mainloop()

    def _build_ui(self):
        outer = ttk.Frame(self.root)
        outer.pack(fill='both', expand=True, padx=8, pady=4)

        top_frame = ttk.Frame(outer)
        top_frame.pack(fill='x', pady=(0, 4))
        
        self._build_file_pickers(top_frame)
        self._build_options(top_frame)
        
        self._build_controls(outer)
        self._build_console(outer)
        self._build_status_bar(outer)

    def _build_file_pickers(self, parent):
        frame = ttk.LabelFrame(parent, text='  Paths  ', padding=8)
        frame.pack(fill='x', pady=(0, 4))

        self.sv_input = self._file_row(frame, 'Input VMF:', 0, 'open', filetypes=[('VMF Files', '*.vmf'), ('All', '*.*')])
        self.sv_game = self._file_row(frame, 'Game Dir:', 1, 'dir')
        self.sv_vbsp = self._file_row(frame, 'VBSP Exe:', 2, 'open', filetypes=[('Executables', '*.exe'), ('All', '*.*')])
        self.sv_vvis = self._file_row(frame, 'VVIS Exe:', 3, 'open', filetypes=[('Executables', '*.exe'), ('All', '*.*')])
        
        frame.columnconfigure(1, weight=1)

    def _file_row(self, parent, label: str, row: int, mode: str, filetypes=None) -> tk.StringVar:
        lbl = ttk.Label(parent, text=label, width=12, anchor='e')
        lbl.grid(row=row, column=0, sticky='e', padx=(0, 6), pady=2)
        sv = tk.StringVar()
        entry = ttk.Entry(parent, textvariable=sv)
        entry.grid(row=row, column=1, sticky='ew', pady=2)

        def browse():
            if mode == 'open':
                p = filedialog.askopenfilename(filetypes=filetypes or [])
            else:
                p = filedialog.askdirectory()
            if p: sv.set(p)

        btn = ttk.Button(parent, text='Browse…', command=browse)
        btn.grid(row=row, column=2, sticky='e', padx=(4, 0), pady=2)
        return sv

    def _build_options(self, parent):
        frame = ttk.LabelFrame(parent, text='  Options  ', padding=6)
        frame.pack(fill='x', pady=(0, 4))

        self.bv_debug = tk.BooleanVar(value=True)
        cb1 = ttk.Checkbutton(frame, text='Debug Material Painter (Paints visible & never-visible faces)', variable=self.bv_debug)
        cb1.pack(anchor='w', pady=1)

        self.bv_fast = tk.BooleanVar(value=True)
        cb2 = ttk.Checkbutton(frame, text='Skip full VVIS (use -fast)', variable=self.bv_fast)
        cb2.pack(anchor='w', pady=1)
        
        bw_frame = ttk.Frame(frame)
        bw_frame.pack(fill='x', pady=2)
        ttk.Label(bw_frame, text='Tiers:').pack(side='left')
        self.sv_tiers = tk.StringVar(value='A,B')
        ttk.Combobox(bw_frame, textvariable=self.sv_tiers, width=5, values=('A,B', 'A', 'B'),
                     state='readonly').pack(side='left', padx=(4, 10))
        ttk.Label(bw_frame, text='Profile:').pack(side='left')
        self.sv_profile = tk.StringVar(value='auto')
        ttk.Combobox(bw_frame, textvariable=self.sv_profile, width=9,
                     values=('auto', 'hl2', 'episodic', 'hl2mp', 'cstrike', 'tf'),
                     state='readonly').pack(side='left', padx=(4, 10))
        ttk.Label(bw_frame, text='Threads:').pack(side='left')
        self.sv_workers = tk.StringVar(value='0')
        ttk.Spinbox(bw_frame, from_=0, to=64, width=5, textvariable=self.sv_workers).pack(side='left', padx=4)

        ba_frame = ttk.Frame(frame)
        ba_frame.pack(fill='x', pady=2)
        ttk.Label(ba_frame, text='Extra bspwalk args:').pack(side='left')
        self.sv_bw_args = tk.StringVar(value='')
        ttk.Entry(ba_frame, textvariable=self.sv_bw_args).pack(side='left', fill='x', expand=True, padx=4)

    def _build_controls(self, parent):
        frame = ttk.Frame(parent)
        frame.pack(fill='x', pady=(4, 2))
        self.btn_run = ttk.Button(frame, text='▶  Run Simulator', style='Run.TButton', command=self._on_run)
        self.btn_run.pack(side='left', padx=(0, 8))
        ttk.Button(frame, text='Clear Log', command=self._clear_log).pack(side='left')
        self.lbl_elapsed = ttk.Label(frame, text='', style='Status.TLabel')
        self.lbl_elapsed.pack(side='right', padx=(8, 0))

    def _build_console(self, parent):
        frame = ttk.LabelFrame(parent, text='  Console Output  ', padding=4)
        frame.pack(fill='both', expand=True, pady=(0, 2))
        self.txt_console = tk.Text(
            frame, wrap='word', state='disabled', bg='#0d1117', fg='#c9d1d9',
            insertbackground=_ACCENT, font=('Consolas', 10), relief='flat',
            selectbackground=_BG3, selectforeground='#ffffff', padx=8, pady=6)
        scrollbar = ttk.Scrollbar(frame, orient='vertical', command=self.txt_console.yview)
        self.txt_console.configure(yscrollcommand=scrollbar.set)
        scrollbar.pack(side='right', fill='y')
        self.txt_console.pack(side='left', fill='both', expand=True)

        self.txt_console.tag_configure('phase', foreground=_ACCENT, font=('Consolas', 10, 'bold'))
        self.txt_console.tag_configure('error', foreground=_RED)

    def _build_status_bar(self, parent):
        bar = ttk.Frame(parent)
        bar.pack(fill='x', pady=(0, 4))
        self.progress = ttk.Progressbar(bar, mode='indeterminate', length=200)
        self.progress.pack(side='left', padx=(0, 8))
        self.lbl_status = ttk.Label(bar, text='Ready', style='Status.TLabel')
        self.lbl_status.pack(side='left', fill='x', expand=True)

    def _auto_detect(self):
        if not self.sv_vbsp.get():
            found = _find_exe('vbsp_lmo.exe')
            if found: self.sv_vbsp.set(found)
        if not self.sv_vvis.get():
            found = _find_exe('vvis_optix.exe')
            if found: self.sv_vvis.set(found)

    def _restore_settings(self):
        s = self._settings
        if 'input' in s: self.sv_input.set(s['input'])
        if 'game' in s: self.sv_game.set(s['game'])
        if 'vbsp' in s: self.sv_vbsp.set(s['vbsp'])
        if 'vvis' in s: self.sv_vvis.set(s['vvis'])
        if 'debug' in s: self.bv_debug.set(s['debug'])
        if 'fast' in s: self.bv_fast.set(s['fast'])
        if 'workers' in s: self.sv_workers.set(s['workers'])
        if 'tiers' in s: self.sv_tiers.set(s['tiers'])
        if 'profile' in s: self.sv_profile.set(s['profile'])
        if 'bw_args' in s: self.sv_bw_args.set(s['bw_args'])

    def _gather_settings(self):
        return {
            'input': self.sv_input.get(),
            'game': self.sv_game.get(),
            'vbsp': self.sv_vbsp.get(),
            'vvis': self.sv_vvis.get(),
            'debug': self.bv_debug.get(),
            'fast': self.bv_fast.get(),
            'workers': self.sv_workers.get(),
            'tiers': self.sv_tiers.get(),
            'profile': self.sv_profile.get(),
            'bw_args': self.sv_bw_args.get(),
        }

    def _on_run(self):
        if self._running:
            self._cancel_run()
            return

        cmd = [sys.executable, str(_SCRIPT_PATH)]

        inp = self.sv_input.get().strip()
        if not inp:
            self._append_console('ERROR: Input VMF is required.\n', 'error')
            return
        cmd.append(inp)

        game = self.sv_game.get().strip()
        if not game:
            self._append_console('ERROR: Game directory is required.\n', 'error')
            return
        cmd.extend(['--game', game])

        if self.sv_vbsp.get().strip(): cmd.extend(['--vbsp', self.sv_vbsp.get().strip()])
        if self.sv_vvis.get().strip(): cmd.extend(['--vvis', self.sv_vvis.get().strip()])
        if self.bv_debug.get(): cmd.append('--debug')
        if self.bv_fast.get(): cmd.append('--vvis-fast')
        cmd.extend(['--workers', self.sv_workers.get()])

        if not (_THIS_DIR / 'bspwalk.exe').is_file():
            self._append_console(f'ERROR: bspwalk.exe not found in {_THIS_DIR}.\n', 'error')
            return
        cmd.extend(['--bspwalk-tiers', self.sv_tiers.get().strip() or 'A,B'])
        bw_args = self.sv_bw_args.get().strip()
        profile = self.sv_profile.get().strip()
        if profile and profile != 'auto' and '-profile' not in bw_args:
            bw_args = f'-profile {profile} {bw_args}'.strip()
        if bw_args:
            cmd.append(f'--bspwalk-args={bw_args}')

        _save_settings(self._gather_settings())
        self._clear_log()
        self._append_console(f'> {" ".join(cmd)}\n\n', 'phase')
        self._set_running(True)

        try:
            self._process = subprocess.Popen(
                cmd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, bufsize=1,
                encoding='utf-8', errors='replace',
                creationflags=subprocess.CREATE_NO_WINDOW if sys.platform == 'win32' else 0
            )
        except Exception as e:
            self._append_console(f'Failed to start process: {e}\n', 'error')
            self._set_running(False)
            return

        self._reader_thread = threading.Thread(target=self._read_output, daemon=True)
        self._reader_thread.start()

    def _cancel_run(self):
        if self._process:
            try:
                if sys.platform == 'win32': self._process.terminate()
                else: os.killpg(os.getpgid(self._process.pid), signal.SIGTERM)
            except Exception: pass
            self._append_console('\n--- Cancelled by user ---\n', 'error')

    def _read_output(self):
        proc = self._process
        try:
            for line in proc.stdout: self._output_queue.put(line)
        except Exception: pass
        finally:
            proc.wait()
            self._output_queue.put(None)

    def _poll_output(self):
        try:
            while True:
                item = self._output_queue.get_nowait()
                if item is None:
                    rc = self._process.returncode if self._process else -1
                    if rc == 0:
                        self._append_console('\n✓ Process completed successfully.\n')
                        self.lbl_status.configure(text='✓ Done', style='StatusOk.TLabel')
                    else:
                        self._append_console(f'\n✗ Process exited with code {rc}.\n', 'error')
                        self.lbl_status.configure(text=f'✗ Exited ({rc})', style='StatusErr.TLabel')
                    self._set_running(False)
                    break
                else:
                    self._append_console(item)
        except queue.Empty: pass

        if self._running:
            elapsed = time.perf_counter() - self._start_time
            self.lbl_elapsed.configure(text=f'{elapsed:.1f}s')

        self.root.after(80, self._poll_output)

    def _append_console(self, text: str, tag: str = ''):
        self.txt_console.configure(state='normal')
        if tag: self.txt_console.insert('end', text, tag)
        else: self.txt_console.insert('end', text)
        self.txt_console.see('end')
        self.txt_console.configure(state='disabled')

    def _clear_log(self):
        self.txt_console.configure(state='normal')
        self.txt_console.delete('1.0', 'end')
        self.txt_console.configure(state='disabled')

    def _set_running(self, running: bool):
        self._running = running
        if running:
            self._start_time = time.perf_counter()
            self.btn_run.configure(text='■  Cancel', style='Cancel.TButton')
            self.lbl_status.configure(text='Running…', style='Status.TLabel')
            self.progress.configure(mode='indeterminate')
            self.progress.start(15)
        else:
            self.btn_run.configure(text='▶  Run Simulator', style='Run.TButton')
            self.progress.stop()
            self.progress.configure(mode='determinate', value=0)
            self._process = None

    def _on_close(self):
        _save_settings(self._gather_settings())
        if self._running: self._cancel_run()
        self.root.destroy()


def main():
    app = VisSimulatorGUI()
    app.run()

if __name__ == '__main__':
    main()
