#!/usr/bin/env python3
"""
lmoptimizer_gui — GUI companion for the VMF Lightmap Scale Optimizer.

Wraps lmoptimizer.py as a subprocess, exposing all CLI arguments through
a polished tkinter form with file pickers, tooltips, live console output,
and persistent settings.
"""
from __future__ import annotations

import json
import queue
import re
import subprocess
import sys
import threading
import time
import tkinter as tk
from tkinter import filedialog, ttk
from pathlib import Path
from typing import Optional

import proc_tree
from bspwalk_runner import BspwalkError, join_args, profile_choices, split_args, validate_setup


# ═══════════════════════════════════════════════════════════════════════════════
#  Settings persistence
# ═══════════════════════════════════════════════════════════════════════════════

_THIS_DIR = Path(__file__).resolve().parent
_SETTINGS_FILE = _THIS_DIR / '.lmoptimizer_gui.json'
_SCRIPT_PATH = _THIS_DIR / 'lmoptimizer.py'


def _load_settings() -> dict:
    """Load saved settings from disk."""
    try:
        return json.loads(_SETTINGS_FILE.read_text(encoding='utf-8'))
    except Exception:
        return {}


def _save_settings(data: dict) -> None:
    """Persist settings to disk."""
    try:
        _SETTINGS_FILE.write_text(
            json.dumps(data, indent=2), encoding='utf-8')
    except Exception:
        pass


# ═══════════════════════════════════════════════════════════════════════════════
#  Tooltip helper
# ═══════════════════════════════════════════════════════════════════════════════

class ToolTip:
    """Mouse-over tooltip for any tkinter widget."""

    def __init__(self, widget: tk.Widget, text: str, delay: int = 400):
        self.widget = widget
        self.text = text
        self.delay = delay
        self._tip_window: Optional[tk.Toplevel] = None
        self._after_id: Optional[str] = None
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


# ═══════════════════════════════════════════════════════════════════════════════
#  Auto-detection helpers
# ═══════════════════════════════════════════════════════════════════════════════

def _find_exe(name: str) -> str:
    """Look for an executable in the script directory."""
    p = _THIS_DIR / name
    return str(p) if p.exists() else ''


def _auto_output(input_path: str) -> str:
    """Generate default output path from input path."""
    if not input_path:
        return ''
    p = Path(input_path)
    return str(p.with_name(p.stem + '_optimized' + p.suffix))


# ═══════════════════════════════════════════════════════════════════════════════
#  Colour palette & theming
# ═══════════════════════════════════════════════════════════════════════════════

_BG = '#1a1a2e'
_BG2 = '#16213e'
_BG3 = '#0f3460'
_FG = '#e0e0e0'
_FG_DIM = '#8899aa'
_ACCENT = '#00b4d8'
_ACCENT2 = '#0077b6'
_GREEN = '#2ecc71'
_RED = '#e74c3c'
_YELLOW = '#f39c12'
_ENTRY_BG = '#22243a'
_ENTRY_FG = '#e0e0e0'
_BTN_BG = '#0f3460'
_BTN_FG = '#e0e0e0'


def _apply_theme(root: tk.Tk):
    """Apply a dark theme to the ttk widgets."""
    style = ttk.Style(root)
    style.theme_use('clam')

    style.configure('.', background=_BG, foreground=_FG,
                    fieldbackground=_ENTRY_BG, borderwidth=0,
                    font=('Segoe UI', 10))
    style.configure('TFrame', background=_BG)
    style.configure('TLabel', background=_BG, foreground=_FG,
                    font=('Segoe UI', 10))
    style.configure('TLabelframe', background=_BG, foreground=_ACCENT,
                    font=('Segoe UI', 10, 'bold'))
    style.configure('TLabelframe.Label', background=_BG, foreground=_ACCENT,
                    font=('Segoe UI', 10, 'bold'))
    style.configure('TEntry', fieldbackground=_ENTRY_BG,
                    foreground=_ENTRY_FG, insertcolor=_ACCENT,
                    font=('Segoe UI', 10))
    style.configure('TButton', background=_BTN_BG, foreground=_BTN_FG,
                    font=('Segoe UI', 10, 'bold'), padding=(10, 4))
    style.map('TButton',
              background=[('active', _ACCENT2), ('pressed', _ACCENT)],
              foreground=[('active', '#ffffff')])
    style.configure('TCheckbutton', background=_BG, foreground=_FG,
                    font=('Segoe UI', 10))
    style.map('TCheckbutton',
              background=[('active', _BG2)],
              foreground=[('active', _FG)])
    style.configure('TRadiobutton', background=_BG, foreground=_FG,
                    font=('Segoe UI', 10))
    style.map('TRadiobutton',
              background=[('active', _BG2)],
              foreground=[('active', _FG)])
    style.configure('TSpinbox', fieldbackground=_ENTRY_BG,
                    foreground=_ENTRY_FG, arrowcolor=_ACCENT,
                    font=('Segoe UI', 10))

    # Accent button for Run
    style.configure('Run.TButton', background=_ACCENT2, foreground='#ffffff',
                    font=('Segoe UI', 11, 'bold'), padding=(16, 6))
    style.map('Run.TButton',
              background=[('active', _ACCENT), ('pressed', _GREEN)],
              foreground=[('active', '#ffffff')])

    # Cancel button
    style.configure('Cancel.TButton', background=_RED, foreground='#ffffff',
                    font=('Segoe UI', 11, 'bold'), padding=(16, 6))
    style.map('Cancel.TButton',
              background=[('active', '#c0392b'), ('pressed', '#a93226')])

    # Progress bar
    style.configure('TProgressbar', troughcolor=_BG2, background=_ACCENT,
                    thickness=6)

    # Status label styles
    style.configure('Status.TLabel', background=_BG2, foreground=_FG_DIM,
                    font=('Segoe UI', 9))
    style.configure('StatusOk.TLabel', background=_BG2, foreground=_GREEN,
                    font=('Segoe UI', 9, 'bold'))
    style.configure('StatusErr.TLabel', background=_BG2, foreground=_RED,
                    font=('Segoe UI', 9, 'bold'))
    style.configure('StatusWarn.TLabel', background=_BG2, foreground=_YELLOW,
                    font=('Segoe UI', 9, 'bold'))


# ═══════════════════════════════════════════════════════════════════════════════
#  Main GUI Application
# ═══════════════════════════════════════════════════════════════════════════════

class LMOptimizerGUI:
    """Main GUI window."""

    # Phase regex: [1/5], [2/6], etc.
    _PHASE_RE = re.compile(r'\[(\d+)/(\d+)\]')
    # lmoptimizer.STATUS_MARKER: the line it prints on every return, '<marker> errors=N'
    _STATUS_RE = re.compile(r'^##lmoptimizer-status errors=(\d+)\s*$')

    def __init__(self):
        self.root = tk.Tk()
        self.root.title('VMF Lightmap Optimizer')
        self.root.geometry('780x860')
        self.root.minsize(640, 700)
        self.root.configure(bg=_BG)
        _apply_theme(self.root)

        # Try to set a window icon
        try:
            self.root.iconbitmap(default='')
        except Exception:
            pass

        self._process: Optional[subprocess.Popen] = None
        self._reader_thread: Optional[threading.Thread] = None
        self._output_queue: queue.Queue = queue.Queue()
        self._running = False
        self._cancelled = False
        self._error_lines = 0
        self._start_time = 0.0

        # Load saved settings
        self._settings = _load_settings()

        # Build UI
        self._build_ui()

        # Populate from saved settings
        self._restore_settings()

        # Auto-detect executables
        self._auto_detect()

        # Set initial BSP row state
        self._on_bsp_toggle()

        # Start polling loop
        self.root.after(100, self._poll_output)

        # Save on close
        self.root.protocol('WM_DELETE_WINDOW', self._on_close)

    def run(self):
        self.root.mainloop()

    # ─── UI Construction ──────────────────────────────────────────────────────

    def _build_ui(self):
        # Main scrollable area
        outer = ttk.Frame(self.root)
        outer.pack(fill='both', expand=True, padx=8, pady=4)

        # Use a canvas for the top config area so it can scroll if needed
        top_frame = ttk.Frame(outer)
        top_frame.pack(fill='x', pady=(0, 4))

        self._build_file_pickers(top_frame)
        self._build_argument_panels(top_frame)
        self._build_controls(outer)
        self._build_console(outer)
        self._build_status_bar(outer)

    # ── File Pickers ──────────────────────────────────────────────────────────

    def _build_file_pickers(self, parent):
        frame = ttk.LabelFrame(parent, text='  Files & Paths  ', padding=8)
        frame.pack(fill='x', pady=(0, 4))

        self.sv_input = self._file_row(
            frame, 'Input VMF:', 0, 'open',
            filetypes=[('VMF Files', '*.vmf'), ('All', '*.*')],
            tooltip='Input VMF file to optimize.',
            on_change=self._on_input_changed,
        )
        self.sv_output = self._file_row(
            frame, 'Output VMF:', 1, 'save',
            filetypes=[('VMF Files', '*.vmf'), ('All', '*.*')],
            tooltip='Output VMF path (default: input_optimized.vmf).',
        )
        self.sv_bsp = self._file_row(
            frame, 'BSP File:', 2, 'open',
            filetypes=[('BSP Files', '*.bsp'), ('All', '*.*')],
            tooltip='Compiled BSP file for lightmap data.\n'
                    'Only used when "Use existing BSP" is checked\n'
                    'in Options (otherwise auto-compiled).',
        )
        # Store BSP row widgets for enable/disable toggling
        self._bsp_row_widgets = [
            w for w in frame.grid_slaves(row=2)
        ]
        self.sv_vbsp = self._file_row(
            frame, 'VBSP Exe:', 3, 'open',
            filetypes=[('Executables', '*.exe'), ('All', '*.*')],
            tooltip='Path to vbsp_lmo.exe with -countverts support.\n'
                    'Auto-detected in the script directory if omitted.',
        )
        self.sv_vrad = self._file_row(
            frame, 'VRAD Exe:', 4, 'open',
            filetypes=[('Executables', '*.exe'), ('All', '*.*')],
            tooltip='Path to vrad_rtx.exe with -countlights support.\n'
                    'Auto-detected if omitted.',
        )
        self.sv_vvis = self._file_row(
            frame, 'VVIS Exe:', 5, 'open',
            filetypes=[('Executables', '*.exe'), ('All', '*.*')],
            tooltip='Path to vvis.exe (auto-detected alongside VBSP).\n'
                    'Required for accurate light counting —\n'
                    'without VIS data, VRAD under-counts surface lights.',
        )
        self.sv_game = self._file_row(
            frame, 'Game Dir:', 6, 'dir',
            tooltip='Game directory for VBSP -game flag\n'
                    '(e.g. Source SDK Base 2013 Multiplayer\\sourcetest). Required when VBSP is used.',
        )
        self.sv_binroot = self._file_row(
            frame, 'Bin Root:', 7, 'dir',
            tooltip='An engine root directory for finding compatible binaries (where hl2.exe or hl2_win64.exe is)\n'
                    '(e.g. steamapps\\common\\Source SDK Base 2013 Multiplayer)\n'
                    "Useful for when the target -game has incompatible binaries (such as Garry's Mod, or a 32bit engine)\n"
                    'Passes as -binroot to all compile tools.',
        )
        self.sv_lights_rad = self._file_row(
            frame, 'Lights .rad:', 9, 'open',
            filetypes=[('RAD Files', '*.rad'), ('All', '*.*')],
            tooltip='Custom .rad lights file forwarded to VRAD\n'
                    'during light counting (e.g. C:\\maps\\custom_lights.rad).\n'
                    'Leave blank to use only the game\'s default lights.rad.',
        )

        frame.columnconfigure(1, weight=1)

    def _file_row(self, parent, label: str, row: int, mode: str,
                  filetypes=None, tooltip: str = '',
                  on_change=None) -> tk.StringVar:
        """Create a label + entry + browse-button row."""
        lbl = ttk.Label(parent, text=label, width=12, anchor='e')
        lbl.grid(row=row, column=0, sticky='e', padx=(0, 6), pady=2)

        sv = tk.StringVar()
        if on_change:
            sv.trace_add('write', lambda *_a: on_change())

        entry = ttk.Entry(parent, textvariable=sv)
        entry.grid(row=row, column=1, sticky='ew', pady=2)

        def browse():
            if mode == 'open':
                p = filedialog.askopenfilename(filetypes=filetypes or [])
            elif mode == 'save':
                p = filedialog.asksaveasfilename(
                    filetypes=filetypes or [],
                    defaultextension='.vmf')
            else:
                p = filedialog.askdirectory()
            if p:
                sv.set(p)

        btn = ttk.TButton(parent, text='Browse…', command=browse) \
            if False else ttk.Button(parent, text='Browse…', command=browse)
        btn.grid(row=row, column=2, sticky='e', padx=(4, 0), pady=2)

        if tooltip:
            ToolTip(entry, tooltip)
            ToolTip(lbl, tooltip)

        return sv

    def _on_input_changed(self):
        """Auto-populate the output field when input changes."""
        inp = self.sv_input.get()
        if inp and not self.sv_output.get():
            self.sv_output.set(_auto_output(inp))

    # ── Argument Panels ───────────────────────────────────────────────────────

    def _build_argument_panels(self, parent):
        # Two-column layout for argument groups
        cols = ttk.Frame(parent)
        cols.pack(fill='x', pady=(0, 4))
        cols.columnconfigure(0, weight=1)
        cols.columnconfigure(1, weight=1)

        # Left column
        left = ttk.Frame(cols)
        left.grid(row=0, column=0, sticky='nsew', padx=(0, 2))

        self._build_budget_group(left)
        self._build_options_group(left)

        # Right column
        right = ttk.Frame(cols)
        right.grid(row=0, column=1, sticky='nsew', padx=(2, 0))

        self._build_scales_group(right)
        self._build_carving_group(right)

    def _build_budget_group(self, parent):
        frame = ttk.LabelFrame(parent, text='  Budgets  ', padding=6)
        frame.pack(fill='x', pady=(0, 4))

        self.sv_vertex_budget = self._spin_row(
            frame, 'Vertex budget:', 0, 1, 999999, 65536,
            'Target vertex limit for budget solver (default: 65536).')
        self.sv_headroom = self._spin_row(
            frame, 'Headroom:', 1, 0, 65536, 3277,
            'Safety margin subtracted from vertex budget.\n'
            'Prevents near-limit crashes with vvis -fast\n'
            '(default: 3277 = 5% of 65536).')
        self.sv_max_lm_dim = self._spin_row(
            frame, 'Max LM dim:', 2, 1, 256, 32,
            "VBSP's g_maxLightmapDimension (default: 32).")
        self.sv_light_budget = self._spin_row(
            frame, 'Light budget:', 3, 0, 999999, 32767,
            'Maximum surface lights in compiled BSP\n'
            '(default: 32767 — GMod limit).\n'
            'Set to 0 to disable light budget enforcement.')

        frame.columnconfigure(1, weight=1)

    def _build_scales_group(self, parent):
        frame = ttk.LabelFrame(parent, text='  Scale Thresholds  ', padding=6)
        frame.pack(fill='x', pady=(0, 4))

        self.sv_dark_threshold = self._spin_row(
            frame, 'Dark threshold:', 0, 0, 255, 5,
            'Max luminance for "uniformly dark" (default: 5.0).',
            float_mode=True)
        self.sv_detail_min_scale = self._spin_row(
            frame, 'Detail min:', 1, 1, 256, 5,
            'Minimum lightmapscale for %detailtype materials (default: 5).')
        self.sv_gradient_tol = self._spin_row(
            frame, 'Gradient tol.:', 2, 0, 10, 0,
            'Monotonic gradient pre-promotion tolerance.\n'
            '0 = disabled, 0.5 = default when enabled.\n'
            'Luminance tolerance for gradient detection.',
            float_mode=True, increment=0.1)

        frame.columnconfigure(1, weight=1)

    def _build_options_group(self, parent):
        frame = ttk.LabelFrame(parent, text='  Options  ', padding=6)
        frame.pack(fill='x', pady=(0, 4))

        self.bv_dry_run = tk.BooleanVar(value=False)
        cb1 = ttk.Checkbutton(frame, text='Dry run (no output)',
                               variable=self.bv_dry_run)
        cb1.pack(anchor='w', pady=1)
        ToolTip(cb1, 'Report statistics without writing output file.')

        self.bv_verbose = tk.BooleanVar(value=False)
        cb2 = ttk.Checkbutton(frame, text='Verbose',
                               variable=self.bv_verbose)
        cb2.pack(anchor='w', pady=1)
        ToolTip(cb2, 'Print per-face classification details.')

        self.bv_strict_coplanar = tk.BooleanVar(value=False)
        cb3 = ttk.Checkbutton(frame, text='Strict coplanar',
                               variable=self.bv_strict_coplanar)
        cb3.pack(anchor='w', pady=1)
        ToolTip(cb3, 'Use strict coplanar unification (require matching\n'
                     'material and texture axes). Default is broad\n'
                     'plane-only grouping for visual coherence.')

        self.bv_visibility_check = tk.BooleanVar(value=False)
        cb4 = ttk.Checkbutton(frame, text='Visibility check',
                               variable=self.bv_visibility_check)
        cb4.pack(anchor='w', pady=1)
        ToolTip(cb4, 'Run bspwalk.exe to identify faces that no player can\n'
                     'ever see, and promote them to max lightmapscale\n'
                     '(saves vertex budget). Needs bspwalk.exe next to\n'
                     'this script and BSP data (--bsp or auto-compile).')

        # bspwalk options (indented under the checkbox)
        bw_frame = ttk.Frame(frame)
        bw_frame.pack(fill='x', padx=(20, 0), pady=1)
        ttk.Label(bw_frame, text='Tiers:').pack(side='left')
        self.sv_bw_tiers = tk.StringVar(value='A,B')
        cmb_t = ttk.Combobox(bw_frame, textvariable=self.sv_bw_tiers, width=5,
                             values=('A,B', 'A', 'B'), state='readonly')
        cmb_t.pack(side='left', padx=(4, 10))
        ToolTip(cmb_t, 'Which bspwalk tiers count as never-visible:\n'
                       '  A   = proven hidden by the VVIS PVS\n'
                       '  B   = hidden by ray sampling (not a proof)\n'
                       '  A,B = both (default)')
        ttk.Label(bw_frame, text='Profile:').pack(side='left')
        self.sv_bw_profile = tk.StringVar(value='auto')
        cmb_p = ttk.Combobox(bw_frame, textvariable=self.sv_bw_profile, width=9,
                             values=profile_choices(), state='readonly')
        cmb_p.pack(side='left', padx=(4, 10))
        ToolTip(cmb_p, 'Player movement profile (bspwalk_profiles.txt).\n'
                       'auto = chosen from the game directory name; a game\n'
                       'directory that matches no profile falls back to hl2\n'
                       '(warned in the log). Pick one for other games.')
        ttk.Label(bw_frame, text='Threads:').pack(side='left')
        self.sv_vis_workers = tk.StringVar(value='0')
        spin_vw = ttk.Spinbox(bw_frame, from_=0, to=64, width=5,
                              textvariable=self.sv_vis_workers)
        spin_vw.pack(side='left', padx=4)
        ToolTip(spin_vw, 'bspwalk worker threads. 0 = all cores.')

        ba_frame = ttk.Frame(frame)
        ba_frame.pack(fill='x', padx=(20, 0), pady=1)
        ttk.Label(ba_frame, text='Extra bspwalk args:').pack(side='left')
        self.sv_bw_args = tk.StringVar(value='')
        ent_ba = ttk.Entry(ba_frame, textvariable=self.sv_bw_args)
        ent_ba.pack(side='left', fill='x', expand=True, padx=4)
        ToolTip(ent_ba, 'Passed to bspwalk.exe (always run with -game). Put\n'
                        'quotes around a value that contains spaces.\n'
                        'See docs/bsp_walker_rebuild.md for options.')

        self.bv_vis_debug = tk.BooleanVar(value=False)
        cb_vd = ttk.Checkbutton(frame, text='Vis debug VMF',
                                 variable=self.bv_vis_debug)
        cb_vd.pack(anchor='w', padx=(20, 0), pady=1)
        ToolTip(cb_vd, 'Paint all faces by visibility classification:\n'
                       '  Visible       → dev/dev_measuregeneric01\n'
                       '  Never-visible → dev/dev_measuregeneric01b\n'
                       'Open the output VMF in Hammer to inspect.')

        # Separator line
        ttk.Separator(frame, orient='horizontal').pack(fill='x', pady=4)

        self.bv_rtx = tk.BooleanVar(value=False)
        cb_rtx = ttk.Checkbutton(frame, text='RTX acceleration',
                                  variable=self.bv_rtx)
        cb_rtx.pack(anchor='w', pady=1)
        ToolTip(cb_rtx, 'Pass -rtx to VRAD during auto-compile for\n'
                        'GPU-accelerated lighting (requires compatible hardware).\n'
                        'Speeds up the lighting data generation step.')

        self.bv_no_cache = tk.BooleanVar(value=False)
        cb_nc = ttk.Checkbutton(frame, text='No compile cache',
                                 variable=self.bv_no_cache)
        cb_nc.pack(anchor='w', pady=1)
        ToolTip(cb_nc, 'Force a fresh VBSP/VVIS/VRAD compile even if a\n'
                       'cached BSP already exists for this VMF.\n'
                       'The cache is normally reused when the VMF is unchanged.')

        self.bv_use_existing_bsp = tk.BooleanVar(value=False)
        cb_bsp = ttk.Checkbutton(frame, text='Use existing BSP',
                                  variable=self.bv_use_existing_bsp,
                                  command=self._on_bsp_toggle)
        cb_bsp.pack(anchor='w', pady=1)
        ToolTip(cb_bsp, 'Read lightmap data from a pre-compiled BSP file\n'
                        'instead of auto-compiling. Check this if you\n'
                        'already have a compiled BSP with accurate lighting.')

    def _build_carving_group(self, parent):
        frame = ttk.LabelFrame(parent, text='  Brush Carving  ', padding=6)
        frame.pack(fill='x', pady=(0, 4))

        self.sv_carve = tk.StringVar(value='none')
        r1 = ttk.Radiobutton(frame, text='None', variable=self.sv_carve,
                              value='none')
        r1.pack(anchor='w', pady=1)
        ToolTip(r1, 'No brush carving.')

        r2 = ttk.Radiobutton(frame, text='Chop (single-cut)',
                              variable=self.sv_carve, value='chop')
        r2.pack(anchor='w', pady=1)
        ToolTip(r2, 'Single-cut carving: split brushes with mixed-uniformity\n'
                    'faces along one axis-aligned plane.')

        r3 = ttk.Radiobutton(frame, text='MultiChop (multi-cut)',
                              variable=self.sv_carve, value='multichop')
        r3.pack(anchor='w', pady=1)
        ToolTip(r3, 'Multi-cut carving: multiple splits per face to isolate\n'
                    'all contiguous uniform regions (implies --chop).')

    def _spin_row(self, parent, label: str, row: int,
                  from_: float, to: float, default,
                  tooltip: str = '', float_mode: bool = False,
                  increment: float = 1) -> tk.StringVar:
        """Create a label + spinbox row."""
        lbl = ttk.Label(parent, text=label, anchor='e')
        lbl.grid(row=row, column=0, sticky='e', padx=(0, 6), pady=2)

        sv = tk.StringVar(value=str(default))
        spin = ttk.Spinbox(
            parent, from_=from_, to=to, textvariable=sv,
            width=10, increment=increment,
            format='%.1f' if float_mode else '%0.0f',
        )
        spin.grid(row=row, column=1, sticky='w', pady=2)

        if tooltip:
            ToolTip(spin, tooltip)
            ToolTip(lbl, tooltip)

        return sv

    # ── Run Controls ──────────────────────────────────────────────────────────

    def _build_controls(self, parent):
        frame = ttk.Frame(parent)
        frame.pack(fill='x', pady=(4, 2))

        self.btn_run = ttk.Button(
            frame, text='▶  Run Optimizer', style='Run.TButton',
            command=self._on_run)
        self.btn_run.pack(side='left', padx=(0, 8))

        self.btn_clear = ttk.Button(
            frame, text='Clear Log', command=self._clear_log)
        self.btn_clear.pack(side='left')

        # Elapsed timer
        self.lbl_elapsed = ttk.Label(frame, text='', style='Status.TLabel')
        self.lbl_elapsed.pack(side='right', padx=(8, 0))

    # ── Console Output ────────────────────────────────────────────────────────

    def _build_console(self, parent):
        frame = ttk.LabelFrame(parent, text='  Console Output  ', padding=4)
        frame.pack(fill='both', expand=True, pady=(0, 2))

        self.txt_console = tk.Text(
            frame, wrap='word', state='disabled',
            bg='#0d1117', fg='#c9d1d9', insertbackground=_ACCENT,
            font=('Consolas', 10), relief='flat', borderwidth=0,
            selectbackground=_BG3, selectforeground='#ffffff',
            padx=8, pady=6,
        )
        scrollbar = ttk.Scrollbar(frame, orient='vertical',
                                  command=self.txt_console.yview)
        self.txt_console.configure(yscrollcommand=scrollbar.set)
        scrollbar.pack(side='right', fill='y')
        self.txt_console.pack(side='left', fill='both', expand=True)

        # Tag for coloured output
        self.txt_console.tag_configure('phase', foreground=_ACCENT,
                                       font=('Consolas', 10, 'bold'))
        self.txt_console.tag_configure('success', foreground=_GREEN)
        self.txt_console.tag_configure('error', foreground=_RED)
        self.txt_console.tag_configure('warning', foreground=_YELLOW)

    # ── Status Bar ────────────────────────────────────────────────────────────

    def _build_status_bar(self, parent):
        bar = ttk.Frame(parent)
        bar.pack(fill='x', pady=(0, 4))

        self.progress = ttk.Progressbar(bar, mode='indeterminate', length=200)
        self.progress.pack(side='left', padx=(0, 8))

        self.lbl_status = ttk.Label(bar, text='Ready', style='Status.TLabel')
        self.lbl_status.pack(side='left', fill='x', expand=True)

    # ─── Auto-detection ───────────────────────────────────────────────────────

    def _auto_detect(self):
        """Auto-fill VBSP and VRAD paths if found in script dir."""
        if not self.sv_vbsp.get():
            found = _find_exe('vbsp_lmo.exe')
            if found:
                self.sv_vbsp.set(found)
        if not self.sv_vrad.get():
            found = _find_exe('vrad_rtx.exe')
            if found:
                self.sv_vrad.set(found)

    def _on_bsp_toggle(self):
        """Enable/disable the BSP file picker row based on 'Use existing BSP'."""
        enabled = self.bv_use_existing_bsp.get()
        state = 'normal' if enabled else 'disabled'
        for w in getattr(self, '_bsp_row_widgets', []):
            try:
                w.configure(state=state)
            except Exception:
                pass  # Not all widgets support state

    # ─── Settings Persistence ─────────────────────────────────────────────────

    def _restore_settings(self):
        """Populate fields from saved settings."""
        s = self._settings
        for key, sv in self._get_string_vars().items():
            if key in s:
                sv.set(s[key])
        for key, bv in self._get_bool_vars().items():
            if key in s:
                bv.set(s[key])

    def _gather_settings(self) -> dict:
        """Collect current field values for persistence."""
        d = {}
        for key, sv in self._get_string_vars().items():
            d[key] = sv.get()
        for key, bv in self._get_bool_vars().items():
            d[key] = bv.get()
        return d

    def _get_string_vars(self) -> dict:
        return {
            'input': self.sv_input,
            'output': self.sv_output,
            'bsp': self.sv_bsp,
            'vbsp': self.sv_vbsp,
            'vrad': self.sv_vrad,
            'vvis': self.sv_vvis,
            'game': self.sv_game,
            'binroot': self.sv_binroot,
            'lights_rad': self.sv_lights_rad,
            'vertex_budget': self.sv_vertex_budget,
            'headroom': self.sv_headroom,
            'max_lm_dim': self.sv_max_lm_dim,
            'light_budget': self.sv_light_budget,
            'dark_threshold': self.sv_dark_threshold,
            'detail_min_scale': self.sv_detail_min_scale,
            'gradient_tol': self.sv_gradient_tol,
            'carve': self.sv_carve,
            'vis_workers': self.sv_vis_workers,
            # _v2: the default became 'A' (#16); a new key drops the 'A,B' every older save holds
            'bw_tiers_v2': self.sv_bw_tiers,
            'bw_profile': self.sv_bw_profile,
            'bw_args': self.sv_bw_args,
        }

    def _get_bool_vars(self) -> dict:
        return {
            'dry_run': self.bv_dry_run,
            'verbose': self.bv_verbose,
            'strict_coplanar': self.bv_strict_coplanar,
            'visibility_check': self.bv_visibility_check,
            'vis_debug': self.bv_vis_debug,
            'rtx': self.bv_rtx,
            'no_cache': self.bv_no_cache,
            'use_existing_bsp': self.bv_use_existing_bsp,
        }

    # ─── Command Building ─────────────────────────────────────────────────────

    def _build_command(self) -> list[str]:
        """Assemble the lmoptimizer.py command line from GUI fields."""
        cmd = [sys.executable, str(_SCRIPT_PATH)]

        # Required: input VMF
        inp = self.sv_input.get().strip()
        if not inp:
            raise ValueError('Input VMF file is required.')
        cmd.append(inp)

        # Output
        out = self.sv_output.get().strip()
        if out:
            cmd.extend(['-o', out])

        # BSP — only passed if "Use existing BSP" is checked
        if self.bv_use_existing_bsp.get():
            bsp = self.sv_bsp.get().strip()
            if bsp:
                cmd.extend(['--bsp', bsp])

        # VBSP
        vbsp = self.sv_vbsp.get().strip()
        if vbsp:
            cmd.extend(['--vbsp', vbsp])

        # VRAD
        vrad = self.sv_vrad.get().strip()
        if vrad:
            cmd.extend(['--vrad', vrad])

        # VVIS
        vvis = self.sv_vvis.get().strip()
        if vvis:
            cmd.extend(['--vvis', vvis])

        # Game dir
        game = self.sv_game.get().strip()
        if game:
            cmd.extend(['--game', game])

        # Bin root dir
        binroot = self.sv_binroot.get().strip()
        if binroot:
            cmd.extend(['--binroot', binroot])

        # Custom lights .rad file
        lights_rad = self.sv_lights_rad.get().strip()
        if lights_rad:
            cmd.extend(['--lights', lights_rad])

        # Budget params
        cmd.extend(['--vertex-budget', self.sv_vertex_budget.get()])
        cmd.extend(['--headroom', self.sv_headroom.get()])
        cmd.extend(['--max-lightmap-dim', self.sv_max_lm_dim.get()])
        cmd.extend(['--light-budget', self.sv_light_budget.get()])

        # Scale params
        cmd.extend(['--dark-threshold', self.sv_dark_threshold.get()])
        cmd.extend(['--detail-min-scale', self.sv_detail_min_scale.get()])

        # Gradient tolerance (0 = omit flag, >0 = pass value)
        gt = float(self.sv_gradient_tol.get() or '0')
        if gt > 0:
            cmd.extend(['--gradient-tolerance', str(gt)])

        # Flags
        if self.bv_dry_run.get():
            cmd.append('--dry-run')
        if self.bv_verbose.get():
            cmd.append('--verbose')
        if self.bv_strict_coplanar.get():
            cmd.append('--strict-coplanar')
        if self.bv_visibility_check.get():
            cmd.append('--visibility-check')
        if self.bv_vis_debug.get():
            cmd.append('--vis-debug')
        if self.bv_rtx.get():
            cmd.append('--rtx')
        if self.bv_no_cache.get():
            cmd.append('--no-cache')

        # bspwalk options
        if self.bv_visibility_check.get():
            tiers = self.sv_bw_tiers.get().strip() or 'A,B'
            try:
                # bspwalk.exe present, Game dir valid, tiers non-empty: refuse before anything runs
                validate_setup(game, exe=_SCRIPT_PATH.parent / 'bspwalk.exe', tiers=tiers)
                bw_args = split_args(self.sv_bw_args.get())
            except BspwalkError as e:
                raise ValueError(f'Visibility check: {e}') from None
            cmd.extend(['--bspwalk-tiers', tiers])
            profile = self.sv_bw_profile.get().strip()
            if profile and profile != 'auto' and not any(a.lower() == '-profile' for a in bw_args):
                bw_args = ['-profile', profile] + bw_args
            if bw_args:
                # one --bspwalk-args=<string> that lmoptimizer splits again: keep each token whole.
                # join_args refuses a token it cannot quote; that is a field error to show, not a crash.
                try:
                    cmd.append('--bspwalk-args=' + join_args(bw_args))
                except BspwalkError as e:
                    raise ValueError(f'Visibility check: {e}') from None
            vw = int(self.sv_vis_workers.get() or '0')
            if vw != 0:
                cmd.extend(['--vis-workers', str(vw)])

        # Carving
        carve = self.sv_carve.get()
        if carve == 'chop':
            cmd.append('--chop')
        elif carve == 'multichop':
            cmd.append('--multichop')

        return cmd

    # ─── Run / Cancel ─────────────────────────────────────────────────────────

    def _on_run(self):
        if self._running:
            self._cancel_run()
            return

        try:
            cmd = self._build_command()
        except ValueError as e:
            self._append_console(f'ERROR: {e}\n', 'error')
            return

        # Save settings before running
        _save_settings(self._gather_settings())

        self._clear_log()
        self._append_console(f'> {" ".join(cmd)}\n\n', 'phase')
        self._set_running(True)

        # Launch the optimizer inside a job object, so Cancel can kill the whole tree
        # (bspwalk, vbsp, vvis, vrad), not just the Python process.
        self._cancelled = False
        self._error_lines = 0
        try:
            self._process = proc_tree.launch(
                cmd,
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                bufsize=1,
                encoding='utf-8',
                errors='replace',
                env=proc_tree.python_child_env(),
            )
        except Exception as e:
            self._append_console(f'Failed to start process: {e}\n', 'error')
            self._set_running(False)
            return

        # Start reader thread
        self._reader_thread = threading.Thread(
            target=self._read_output, daemon=True)
        self._reader_thread.start()

    def _cancel_run(self):
        if self._process and not self._cancelled:
            self._cancelled = True
            proc_tree.kill_tree_async(self._process)   # taskkill can take seconds: not on the Tk thread
            self.lbl_status.configure(text='Cancelling…', style='Status.TLabel')
            self._append_console(
                '\n--- Cancel requested (stopping the optimizer and its child tools) ---\n', 'warning')

    def _read_output(self):
        """Background thread: read subprocess output line by line."""
        proc = self._process
        try:
            for line in proc.stdout:
                self._output_queue.put(line)
        except Exception:
            pass
        finally:
            proc.wait()
            self._output_queue.put(None)  # sentinel

    def _poll_output(self):
        """Main-thread polling: drain queue into console."""
        try:
            while True:
                item = self._output_queue.get_nowait()
                if item is None:
                    if proc_tree.kill_pending(self._process):
                        self._output_queue.put(None)   # tree kill still running: look again next tick
                        break
                    # Process finished
                    rc = self._process.returncode if self._process else None
                    if self._cancelled:
                        self.lbl_status.configure(
                            text='✗ Cancelled', style='StatusErr.TLabel')
                    elif rc == 0 and self._error_lines:
                        # defensive: every ERROR lmoptimizer reports today exits nonzero
                        self._append_console(
                            f'\n⚠ Process exited 0 but logged {self._error_lines} '
                            f'error line(s); check the log above.\n', 'warning')
                        self.lbl_status.configure(
                            text='⚠ Finished with errors logged',
                            style='StatusWarn.TLabel')
                    elif rc == 0:
                        self._append_console(
                            '\n✓ Process completed successfully.\n', 'success')
                        self.lbl_status.configure(
                            text='✓ Done', style='StatusOk.TLabel')
                    else:
                        self._append_console(
                            f'\n✗ Process exited with code {rc}.\n', 'error')
                        self.lbl_status.configure(
                            text=f'✗ Exited ({rc})', style='StatusErr.TLabel')
                    self._set_running(False)
                    break
                else:
                    self._process_line(item)
        except queue.Empty:
            pass

        # Update elapsed timer
        if self._running:
            elapsed = time.perf_counter() - self._start_time
            self.lbl_elapsed.configure(text=f'{elapsed:.1f}s')

        self.root.after(80, self._poll_output)

    def _process_line(self, line: str):
        """Classify and display a single output line."""
        stripped = line.rstrip('\n\r')
        status = self._STATUS_RE.match(stripped)
        if status:   # machine-readable trailer: counted, not shown
            self._error_lines = int(status.group(1))
            return
        if stripped.strip().startswith('Running bspwalk'):
            # no [n/m] marker follows during the walk: keep the bar moving, not frozen at 4/6
            self.lbl_status.configure(text='Running bspwalk…', style='Status.TLabel')
            self.progress.configure(mode='indeterminate')
            self.progress.start(15)

        # Detect phase markers
        m = self._PHASE_RE.search(stripped)
        if m:
            phase, total = int(m.group(1)), int(m.group(2))
            self.lbl_status.configure(
                text=f'Phase {phase}/{total}: {stripped.split("]", 1)[-1].strip()}',
                style='Status.TLabel')
            self.progress.stop()
            self.progress.configure(
                mode='determinate', maximum=total, value=phase)
            self._append_console(line, 'phase')
            return

        # Detect success/failure markers
        if '✓' in stripped or 'UNDER BUDGET' in stripped:
            self._append_console(line, 'success')
        elif '✗' in stripped or 'ERROR' in stripped or 'OVER BUDGET' in stripped:
            self._append_console(line, 'error')
        elif '⚠' in stripped or 'WARNING' in stripped:
            self._append_console(line, 'warning')
        else:
            self._append_console(line)

    # ─── Console Helpers ──────────────────────────────────────────────────────

    def _append_console(self, text: str, tag: str = ''):
        self.txt_console.configure(state='normal')
        if tag:
            self.txt_console.insert('end', text, tag)
        else:
            self.txt_console.insert('end', text)
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
            self.lbl_status.configure(
                text='Running…', style='Status.TLabel')
            self.progress.configure(mode='indeterminate')
            self.progress.start(15)
        else:
            self.btn_run.configure(
                text='▶  Run Optimizer', style='Run.TButton')
            self.progress.stop()
            self.progress.configure(mode='determinate', value=0)
            proc_tree.release(self._process)
            self._process = None

    # ─── Lifecycle ────────────────────────────────────────────────────────────

    def _on_close(self):
        _save_settings(self._gather_settings())
        if self._running:
            self._cancel_run()
        self.root.destroy()


# ═══════════════════════════════════════════════════════════════════════════════

if __name__ == '__main__':
    app = LMOptimizerGUI()
    app.run()
