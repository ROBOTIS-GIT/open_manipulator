#!/usr/bin/env python3
"""
OMX Port Identifier — identifies Leader / Follower arms and Camera,
generates a ready-to-paste lerobot-record command.

Run on HOST (not inside Docker):
  sudo apt install python3-tk
  python3 omx_port_identifier.py
"""

import tkinter as tk
from tkinter import font as tkfont, messagebox
import threading, os, re, glob, json, subprocess, sys

REGISTRY_PATH = os.path.expanduser("~/.config/omx_port_id/boards.json")
ROLES         = ["Leader", "Follower"]

# Palette
BG      = "#f4f6f8"
SURFACE = "#eaeff4"
BORDER  = "#cbd5e1"
TEXT    = "#0f172a"
MUTED   = "#374151"
SUCCESS = "#059669"
DANGER  = "#dc2626"
INFO    = "#2563eb"
WARN    = "#d97706"

ROLE_FG     = {"Leader": "#1e3a8a", "Follower": "#064e3b", "Camera": "#581c87"}
ROLE_SURF   = {"Leader": "#dbeafe", "Follower": "#d1fae5", "Camera": "#ede9fe"}
ROLE_BORDER = {"Leader": "#3b82f6", "Follower": "#10b981", "Camera": "#8b5cf6"}


def serial_devices():
    d = set(glob.glob("/dev/serial/by-id/*"))
    if not d:
        d  = set(glob.glob("/dev/ttyACM*"))
        d |= set(glob.glob("/dev/ttyUSB*"))
    return d

def hw_serial(port):
    if "/dev/serial/by-id/" not in port:
        real = os.path.realpath(port)
        m = [p for p in glob.glob("/dev/serial/by-id/*")
             if os.path.realpath(p) == real]
        if m: port = m[0]
        else: return None
    name = re.sub(r'-if\d+$', '', os.path.basename(port))
    parts = name.split('_')
    return parts[-1] if len(parts) > 1 else None

def short_port(port):
    base = re.sub(r'-if\d+$', '', os.path.basename(port))
    parts = base.split('_')
    serial = parts[-1] if len(parts) > 1 else base
    return serial[:20] + '...' if len(serial) > 20 else serial

def usb_slot(port):
    try:
        r = subprocess.run(['udevadm', 'info', '-q', 'property', '-n',
                            os.path.realpath(port)],
                           capture_output=True, text=True, timeout=2)
        for line in r.stdout.splitlines():
            if line.startswith('ID_PATH='):
                m = re.search(r'usb-[\d.]+:([\d.]+)', line)
                if m: return f"USB {m.group(1)}"
    except Exception: pass
    return ""

def udev_props(path):
    try:
        r = subprocess.run(['udevadm', 'info', '-q', 'property', '-n',
                            os.path.realpath(path)],
                           capture_output=True, text=True, timeout=2)
        return r.stdout
    except Exception: return ""

def camera_info(video_path):
    m = re.search(r'video(\d+)$', video_path)
    dev_num = int(m.group(1)) if m else 0
    dev = os.path.basename(os.path.realpath(video_path))
    name = "Camera"
    try:
        with open(f"/sys/class/video4linux/{dev}/name") as f:
            name = f.read().strip()
    except Exception: pass
    props = udev_props(video_path)
    vendor = ""
    for line in props.splitlines():
        if line.startswith("ID_VENDOR_FROM_DATABASE="):
            vendor = line.split("=", 1)[1].strip()
    usb_parent = ""
    try:
        usb_parent = os.path.dirname(
            os.path.realpath(f"/sys/class/video4linux/{dev}/device"))
    except Exception: pass
    return {"path": video_path, "name": name, "vendor": vendor,
            "opencv_idx": dev_num, "usb_parent": usb_parent}

def safe_usb_cameras():
    try:
        video_paths = []
        for p in glob.glob("/dev/video*"):
            m = re.search(r'video(\d+)$', p)
            if m:
                video_paths.append((int(m.group(1)), p))
        video_paths.sort()
        all_video = [p for _, p in video_paths]
    except Exception: return []
    seen, result = set(), []
    for path in all_video:
        try:
            info   = camera_info(path)
            parent = info["usb_parent"]
            if parent and parent in seen: continue
            if parent: seen.add(parent)
            result.append(info)
        except Exception: continue
    return result

def load_registry():
    try:
        with open(REGISTRY_PATH) as f: return json.load(f)
    except Exception: return {}

def save_registry(d):
    os.makedirs(os.path.dirname(REGISTRY_PATH), exist_ok=True)
    with open(REGISTRY_PATH, 'w') as f: json.dump(d, f, indent=2)


class App:
    def __init__(self, root):
        self.root = root
        self.root.title("OMX Port Identifier")
        self.root.configure(bg=BG)
        self.root.minsize(900, 640)

        # Font selection: prefer Noto Sans (sharpest on Ubuntu),
        # then DejaVu Sans (always on Ubuntu), then system default.
        # No fractional scaling — we use larger absolute sizes instead
        # to keep rendering crisp and pixel-aligned.
        families = set(tkfont.families())
        sans = next((f for f in [
            "Noto Sans", "Ubuntu", "DejaVu Sans",
            "Liberation Sans", "Arial", "Helvetica"]
            if f in families),
            tkfont.nametofont("TkDefaultFont").actual()["family"])
        mono = next((f for f in [
            "Noto Mono", "Ubuntu Mono", "DejaVu Sans Mono",
            "Liberation Mono", "Courier New"]
            if f in families),
            tkfont.nametofont("TkFixedFont").actual()["family"])

        self.F = {
            "title":   tkfont.Font(family=sans, size=24, weight="bold"),
            "instr":   tkfont.Font(family=sans, size=20, weight="bold"),
            "instr_s": tkfont.Font(family=sans, size=16),
            "h2":      tkfont.Font(family=sans, size=16, weight="bold"),
            "body":    tkfont.Font(family=sans, size=16),
            "small":   tkfont.Font(family=sans, size=15),
            "mono":    tkfont.Font(family=mono, size=15),
            "btn":     tkfont.Font(family=sans, size=16, weight="bold"),
            "big":     tkfont.Font(family=sans, size=19, weight="bold"),
        }

        self.assignments = {}
        self.port_data   = {}
        self.registry    = load_registry()
        self._stop_watch = False
        self._lerobot_expanded = False
        self._pulse_job  = None

        # Launch file paths
        _ws = os.path.expanduser(
            '~/robot_ws/open_manipulator/open_manipulator_bringup/launch')
        self._leader_launch   = tk.StringVar(
            value=os.path.join(_ws, 'omx_l_leader_ai.launch.py'))
        self._follower_launch = tk.StringVar(
            value=os.path.join(_ws, 'omx_f_follower_ai.launch.py'))
        self._apply_result = tk.StringVar(value='')

        self._hf_user_var  = tk.StringVar(value="")
        self._dataset_var  = tk.StringVar(value="omx-test")
        self._cam_idx_var  = tk.StringVar(value="2")
        self._episodes_var = tk.StringVar(value="5")
        self._task_var     = tk.StringVar(value="Pick up the object")
        self._display_var  = tk.StringVar(value="true")

        self._build()
        threading.Thread(target=self._run_detection, daemon=True).start()

    # ── Build ─────────────────────────────────────────────────────────────────

    def _build(self):
        # Header
        hdr = tk.Frame(self.root, bg=BG,
                       highlightbackground=BORDER, highlightthickness=1)
        hdr.pack(fill="x")
        hdr_in = tk.Frame(hdr, bg=BG, padx=22, pady=14)
        hdr_in.pack(fill="x")
        tk.Label(hdr_in, text="OMX Port Identifier",
                 bg=BG, fg=TEXT, font=self.F["title"]).pack(side="left")
        self._hdr_lbl = tk.Label(hdr_in, text="Scanning...",
                                  bg=BG, fg="#374151", font=self.F["body"])
        self._hdr_lbl.pack(side="left", padx=16)

        # ── Instruction banner (prominent, at the top) ─────────────────────
        self._instr_frame = tk.Frame(self.root, bg=SURFACE,
                                     highlightbackground=BORDER,
                                     highlightthickness=1)
        self._instr_frame.pack(fill="x")

        instr_in = tk.Frame(self._instr_frame, bg=SURFACE, padx=22, pady=14)
        instr_in.pack(fill="x")

        self._instr_title = tk.Label(
            instr_in, text="Scanning...",
            bg=SURFACE, fg=INFO,
            font=self.F["instr"], anchor="w")
        self._instr_title.pack(fill="x")

        self._instr_sub = tk.Label(
            instr_in, text="",
            bg=SURFACE, fg="#374151",
            font=self.F["instr_s"], anchor="w", justify="left")
        self._instr_sub.pack(fill="x", pady=(4, 0))

        # Watching indicator (pulsing dot + text, shown during active watch)
        self._watch_row = tk.Frame(instr_in, bg=SURFACE)
        self._dot_canvas = tk.Canvas(self._watch_row, width=14, height=14,
                                     bg=SURFACE, highlightthickness=0)
        self._dot_canvas.pack(side="left", padx=(0, 8))
        self._dot = self._dot_canvas.create_oval(2, 2, 12, 12,
                                                  fill=BORDER, outline="")
        self._watch_lbl = tk.Label(self._watch_row, text="",
                                    bg=SURFACE, fg=MUTED,
                                    font=self.F["small"])
        self._watch_lbl.pack(side="left")

        # Body
        body = tk.Frame(self.root, bg=BG, padx=16, pady=12)
        body.pack(fill="both", expand=True)
        body.columnconfigure(0, weight=8)
        body.columnconfigure(1, weight=2)
        body.rowconfigure(0, weight=1)

        # LEFT
        left = tk.Frame(body, bg=BG)
        left.grid(row=0, column=0, sticky="nsew", padx=(0, 14))
        left.columnconfigure(0, weight=1)
        left.rowconfigure(1, weight=1)
        tk.Label(left, text="Detected Devices",
                 bg=BG, fg=TEXT, font=self.F["h2"],
                 anchor="w").grid(row=0, column=0, sticky="nw", pady=(0, 8))
        wrap = tk.Frame(left, bg=BG)
        wrap.grid(row=1, column=0, sticky="nsew")
        wrap.columnconfigure(0, weight=1)
        wrap.rowconfigure(0, weight=1)
        self._canvas = tk.Canvas(wrap, bg=BG, highlightthickness=0)
        vsb = tk.Scrollbar(wrap, orient="vertical",
                           command=self._canvas.yview)
        self._canvas.configure(yscrollcommand=vsb.set)
        vsb.grid(row=0, column=1, sticky="ns")
        self._canvas.grid(row=0, column=0, sticky="nsew")
        self._portlist = tk.Frame(self._canvas, bg=BG)
        self._win = self._canvas.create_window(
            (0, 0), window=self._portlist, anchor="nw")
        self._portlist.bind("<Configure>", lambda e: (
            self._canvas.configure(
                scrollregion=self._canvas.bbox("all")),
            self._canvas.itemconfigure(
                self._win, width=self._canvas.winfo_width())))
        self._canvas.bind("<Configure>",
            lambda e: self._canvas.itemconfigure(
                self._win, width=e.width))

        # RIGHT
        right = tk.Frame(body, bg=BG)
        right.grid(row=0, column=1, sticky="nsew")
        right.columnconfigure(0, weight=1)

        # Connection status
        sc = tk.Frame(right, bg=SURFACE,
                      highlightbackground=BORDER, highlightthickness=1)
        sc.grid(row=0, column=0, sticky="nsew")
        right.rowconfigure(0, weight=1)
        tk.Label(sc, text="Connection Status",
                 bg=SURFACE, fg=TEXT, font=self.F["h2"],
                 padx=16, pady=12, anchor="w").pack(fill="x")
        si = tk.Frame(sc, bg=SURFACE, padx=16)
        si.pack(fill="x")
        self._status_widgets = {}
        for role in ("Leader", "Follower", "Camera"):
            f = tk.Frame(si, bg=SURFACE)
            f.pack(fill="x", pady=12)
            dc = tk.Canvas(f, width=14, height=14,
                           bg=SURFACE, highlightthickness=0)
            dc.pack(side="left", padx=(0, 10))
            dot = dc.create_oval(2, 2, 12, 12, fill=BORDER, outline="")
            tk.Label(f, text=role, bg=SURFACE, fg=ROLE_FG[role],
                     font=self.F["btn"], width=8,
                     anchor="w").pack(side="left")
            plbl = tk.Label(f, text="not connected",
                            bg=SURFACE, fg=MUTED, font=self.F["body"])
            plbl.pack(side="left")
            self._status_widgets[role] = (dc, dot, plbl)
        tk.Frame(sc, height=10, bg=SURFACE).pack()

        # Ready card
        self._ready_card = tk.Frame(right, bg="#d1fae5",
                                    highlightbackground="#10b981",
                                    highlightthickness=2)
        tk.Label(self._ready_card,
                 text="All devices identified",
                 bg="#d1fae5", fg="#064e3b",
                 font=self.F["big"],
                 padx=16, pady=12, anchor="w").pack(fill="x")
        tk.Label(self._ready_card,
                 text="Expand the command panel below,\n"
                      "enter your HF username, and copy.",
                 bg="#d1fae5", fg="#065f46",
                 font=self.F["small"],
                 padx=16, pady=4,
                 anchor="w", justify="left").pack(fill="x")
        tk.Frame(self._ready_card, height=10, bg="#d1fae5").pack()

        # Launch file panel
        self._build_launch_panel(right)

        # lerobot panel
        self._build_lerobot_panel()

        # Footer
        foot = tk.Frame(self.root, bg=SURFACE,
                        highlightbackground=BORDER, highlightthickness=1,
                        pady=10)
        foot.pack(fill="x", side="bottom")
        tk.Button(foot, text="Re-scan",
                  command=self._rescan,
                  bg=SURFACE, fg=TEXT, font=self.F["small"],
                  relief="flat", padx=16, pady=7, cursor="hand2",
                  highlightbackground=BORDER,
                  highlightthickness=1).pack(side="left", padx=(18, 6))
        tk.Label(foot, text="Press R to re-scan",
                 bg=SURFACE, fg=MUTED,
                 font=self.F["small"]).pack(side="left", padx=6)
        tk.Button(foot, text="Forget Boards",
                  command=self._forget,
                  bg=SURFACE, fg=DANGER, font=self.F["small"],
                  relief="flat", padx=16, pady=7, cursor="hand2",
                  highlightbackground=BORDER,
                  highlightthickness=1).pack(side="right", padx=(6, 18))
        self.root.bind("<r>", lambda e: self._rescan())
        self.root.bind("<R>", lambda e: self._rescan())

    # ── Instruction banner helpers ─────────────────────────────────────────────

    def _set_instr(self, title, sub="", color=INFO,
                   watching=False, watch_text="Watching for changes..."):
        """Update the top instruction banner."""
        self._instr_title.configure(text=title, fg=color)
        self._instr_sub.configure(text=sub)
        self._hdr_lbl.configure(text=title)
        if watching:
            self._watch_lbl.configure(text=watch_text)
            self._watch_row.pack(fill="x", pady=(6, 0))
            self._start_pulse(color)
        else:
            self._watch_row.pack_forget()
            self._stop_pulse()

    def _start_pulse(self, color):
        self._stop_pulse()
        self._pulse_state = [True]
        def pulse():
            if not self._pulse_state[0]: return
            on = getattr(self, '_pulse_on', True)
            self._pulse_on = not on
            self._dot_canvas.itemconfig(
                self._dot, fill=color if on else SURFACE)
            self._pulse_job = self.root.after(600, pulse)
        pulse()

    def _stop_pulse(self):
        if self._pulse_job:
            self.root.after_cancel(self._pulse_job)
            self._pulse_job = None
        self._pulse_state = [False]
        self._dot_canvas.itemconfig(self._dot, fill=BORDER)

    # ── Status dots ───────────────────────────────────────────────────────────

    def _update_status(self, role, port=None):
        if role not in self._status_widgets: return
        dc, dot, lbl = self._status_widgets[role]
        if port:
            dc.itemconfig(dot, fill=ROLE_BORDER[role])
            if role == "Camera" and port in self.port_data:
                idx = self.port_data[port].get("opencv_idx", "?")
                lbl.configure(text=f"index {idx}", fg=ROLE_FG[role])
            else:
                lbl.configure(text=short_port(port), fg=ROLE_FG[role])
        else:
            dc.itemconfig(dot, fill=BORDER)
            lbl.configure(text="not connected", fg=MUTED)

    def _check_ready(self):
        has_l = any(r == "Leader"   for r in self.assignments.values())
        has_f = any(r == "Follower" for r in self.assignments.values())
        has_c = any(r == "Camera"   for r in self.assignments.values())
        if has_l and has_f and has_c:
            self._ready_card.grid(row=1, column=0,
                                  sticky="new", pady=(12, 0))
        else:
            self._ready_card.grid_forget()

    # ── Detection ─────────────────────────────────────────────────────────────

    def _run_detection(self):
        try:
            self._ui(self._set_instr, "Scanning...",
                     "Checking connected devices...", INFO)
            serials  = sorted(serial_devices())
            cam_info = safe_usb_cameras()
            self._ui(self._rebuild_cards, serials, cam_info)

            found, unknown = {}, []
            for port in serials:
                sid = hw_serial(port)
                if sid and sid in self.registry:
                    found[port] = self.registry[sid]
                    self._ui(self._badge, port, self.registry[sid], "saved")
                else:
                    unknown.append(port)

            if found and not unknown:
                self._ui(self._finish, found,
                         "Recognized automatically",
                         "Both arms matched from saved registry.")
                return

            if found:
                self._ui(self._apply_assignments, found)

            remaining = [r for r in ROLES if r not in found.values()]
            n = len(serials)

            if n == 0:
                self._ui(self._set_instr,
                         "No arms detected",
                         "Plug in both arms to get started.", WARN)
                self._ui(self._flow_plugin, remaining)
            elif n == 1 and unknown:
                self._ui(self._set_instr,
                         "Which arm is this?",
                         "Select its role using the buttons on the card below.",
                         WARN)
                self._ui(self._flow_one_arm, unknown[0], remaining)
            elif n == 1 and not unknown:
                known_role = list(found.values())[0]
                other_role = [r for r in ROLES if r != known_role][0]
                self._ui(self._set_instr,
                         f"Plug in the {other_role} arm",
                         f"{known_role} is recognized. Waiting for {other_role} to connect.",
                         INFO, True,
                         "Watching for the second arm...")
                self._ui(self._wait_for_plug_in,
                         serial_devices(), [other_role])
            else:
                if remaining:
                    self._ui(self._set_instr,
                             f"Unplug the {remaining[0].upper()} arm briefly",
                             "The arm that disappears will be identified and saved.\n"
                             "Plug it back in right after.",
                             WARN, True,
                     "Watching for disconnection...")
                    self._ui(self._flow_unplug, unknown, remaining)
        except Exception as exc:
            self._ui(self._set_instr, f"Scan error: {exc}",
                     "Press R to try again.", DANGER)

    def _finish(self, assignments, title, sub):
        self._apply_assignments(assignments)
        for port, role in assignments.items():
            sid = hw_serial(port)
            if sid and sid not in self.registry:
                self.registry[sid] = role
        save_registry(self.registry)
        self._set_instr(title, sub, SUCCESS)

    # ── Flows ─────────────────────────────────────────────────────────────────

    def _flow_plugin(self, remaining):
        if not remaining: return
        role = remaining[0]
        self._set_instr(
            f"Plug in the {role.upper()} arm now",
            "The arm that appears will be assigned this role automatically.",
            WARN, True,
                     "Watching for connection...")
        before = set(serial_devices())

        def watch(n=0):
            if self._stop_watch: return
            new_devs = serial_devices() - before
            if new_devs:
                port = sorted(new_devs)[0]
                self._assign_port(port, role)
                self._ui(self._rebuild_cards,
                         sorted(serial_devices()), safe_usb_cameras())
                next_r = [r for r in remaining if r != role]
                if next_r:
                    self.root.after(800, lambda: self._flow_plugin(next_r))
                else:
                    self._set_instr("Both arms identified",
                                    "Saved to registry. Future runs are fully automatic.",
                                    SUCCESS)
                return
            if n >= 300:
                self._set_instr("Timed out", "Press R to retry.", DANGER)
                return
            self.root.after(200, lambda: watch(n + 1))
        self.root.after(300, lambda: watch(0))

    def _wait_for_plug_in(self, before, remaining):
        if not remaining: return
        role = remaining[0]

        def watch(n=0):
            if self._stop_watch: return
            new_devs = serial_devices() - before
            if new_devs:
                port = sorted(new_devs)[0]
                self._assign_port(port, role)
                self._ui(self._rebuild_cards,
                         sorted(serial_devices()), safe_usb_cameras())
                next_r = [r for r in remaining if r != role]
                if next_r:
                    self.root.after(800, lambda: self._wait_for_plug_in(
                        serial_devices(), next_r))
                else:
                    self._set_instr("Both arms identified",
                                    "Saved to registry. Future runs are fully automatic.",
                                    SUCCESS)
                return
            if n >= 300:
                self._set_instr("Timed out", "Press R to retry.", DANGER)
                return
            self.root.after(200, lambda: watch(n + 1))
        self.root.after(300, lambda: watch(0))

    def _flow_one_arm(self, port, remaining):
        card = None
        for w in self._portlist.winfo_children():
            if getattr(w, '_role_card', False):
                try: w.destroy()
                except Exception: pass

        card = tk.Frame(self._portlist, bg=SURFACE,
                        highlightbackground=BORDER, highlightthickness=2)
        card._role_card = True
        card.pack(fill="x", pady=(12, 0), ipady=4)
        tk.Label(card, text=short_port(port),
                 bg=SURFACE, fg=MUTED, font=self.F["body"],
                 padx=16, pady=8, anchor="w").pack(fill="x")
        br = tk.Frame(card, bg=SURFACE, padx=16, pady=10)
        br.pack(fill="x")

        def choose(role):
            complement = next((r for r in remaining if r != role), None)
            self._assign_port(port, role)
            try: card.destroy()
            except Exception: pass
            if complement:
                self._set_instr(
                    f"Now plug in the {complement} arm",
                    f"{role} identified. Waiting for {complement} to connect.",
                    INFO, True,
                    "Watching for connection...")
                self._wait_for_plug_in(serial_devices(), [complement])
            else:
                self._set_instr("Both arms identified",
                                "Saved to registry.", SUCCESS)

        for role in ROLES:
            tk.Button(br, text=f"This is the {role}",
                      command=lambda r=role: choose(r),
                      bg=ROLE_SURF[role], fg=ROLE_FG[role],
                      font=self.F["btn"], relief="flat",
                      padx=20, pady=10, cursor="hand2").pack(
                          side="left", padx=(0, 12))
        tk.Frame(card, height=6, bg=SURFACE).pack()

    def _flow_unplug(self, ports, remaining):
        if not remaining or not ports: return
        role = remaining[0]
        self._set_instr(
            f"Unplug the {role.upper()} arm briefly",
            "The arm that disconnects will be identified automatically.\n"
            "Plug it back in right after.",
            WARN, True,
                     "Watching for disconnection...")
        before = set(ports)

        def watch(n=0):
            if self._stop_watch: return
            gone = before - serial_devices()
            if gone:
                port = sorted(gone)[0]
                self._assign_port(port, role)
                next_r = [r for r in remaining if r != role]
                next_p = [p for p in ports if p != port]
                if next_r and len(next_p) == 1:
                    self.root.after(800,
                        lambda: self._auto_confirm(next_p[0], next_r[0]))
                elif next_r and next_p:
                    self.root.after(800,
                        lambda: self._flow_unplug(next_p, next_r))
                else:
                    self._set_instr("Both arms identified",
                                    "Saved to registry. Future runs are fully automatic.",
                                    SUCCESS)
                return
            if n >= 150:
                self._set_instr("Timed out", "Press R to retry.", DANGER)
                return
            self.root.after(200, lambda: watch(n + 1))
        self.root.after(300, lambda: watch(0))

    def _auto_confirm(self, port, role):
        self._set_instr(
            f"Is the remaining arm the {role}?",
            f"One unidentified port remains. Confirm below or cancel to re-identify.",
            INFO)
        for w in self._portlist.winfo_children():
            if getattr(w, '_confirm_card', False):
                try: w.destroy()
                except Exception: pass
        card = tk.Frame(self._portlist, bg=ROLE_SURF[role],
                        highlightbackground=ROLE_BORDER[role],
                        highlightthickness=2)
        card._confirm_card = True
        card.pack(fill="x", pady=(12, 0), ipady=4)
        tk.Label(card, text=short_port(port),
                 bg=ROLE_SURF[role], fg=ROLE_FG[role],
                 font=self.F["body"], padx=16, pady=8,
                 anchor="w").pack(fill="x")
        br = tk.Frame(card, bg=ROLE_SURF[role], padx=16, pady=10)
        br.pack(fill="x")

        def confirm():
            self._assign_port(port, role)
            try: card.destroy()
            except Exception: pass
            self._set_instr("Both arms identified",
                            "Saved to registry. Future runs are fully automatic.",
                            SUCCESS)

        tk.Button(br, text=f"Yes, this is the {role}",
                  command=confirm,
                  bg=ROLE_BORDER[role], fg="#ffffff",
                  font=self.F["btn"], relief="flat",
                  padx=16, pady=10, cursor="hand2").pack(
                      side="left", padx=(0, 12))
        tk.Button(br, text="Cancel",
                  command=lambda: (card.destroy(),
                                   self._flow_unplug([port], [role])),
                  bg=SURFACE, fg=MUTED,
                  font=self.F["small"], relief="flat",
                  padx=14, pady=10, cursor="hand2",
                  highlightbackground=BORDER,
                  highlightthickness=1).pack(side="left")

    # ── Port cards ────────────────────────────────────────────────────────────

    def _rebuild_cards(self, serials, cam_info):
        for w in self._portlist.winfo_children():
            w.destroy()
        self.port_data.clear()

        if not serials and not cam_info:
            tk.Label(self._portlist,
                     text="No USB devices found.\nPlug in your arms to get started.",
                     bg=BG, fg=MUTED, font=self.F["body"],
                     pady=20, justify="left").pack(anchor="w")
            return

        def section(text):
            f = tk.Frame(self._portlist, bg=BG)
            f.pack(fill="x", pady=(12, 4))
            tk.Label(f, text=text, bg=BG, fg=MUTED,
                     font=self.F["small"]).pack(side="left")
            tk.Frame(f, bg=BORDER, height=1).pack(
                side="left", fill="x", expand=True, padx=(10, 0), pady=8)

        if serials:
            section("Arms")
            for p in serials:
                self._make_arm_card(p)

        if cam_info:
            section("Cameras")
            for info in cam_info:
                try: self._make_camera_card(info)
                except Exception: pass

        for port, role in self.assignments.items():
            if port in self.port_data:
                self._refresh_highlight(port, role)
                self._badge(port, role, "identified")

    def _make_arm_card(self, port):
        full  = os.path.basename(port)
        short = short_port(port)
        slot  = usb_slot(port)

        card = tk.Frame(self._portlist, bg=SURFACE,
                        highlightbackground=BORDER, highlightthickness=1)
        card.pack(fill="x", pady=5, ipady=2)

        top = tk.Frame(card, bg=SURFACE, padx=16, pady=10)
        top.pack(fill="x")
        nc = tk.Frame(top, bg=SURFACE)
        nc.pack(side="left", fill="x", expand=True)
        # Show only the short serial — hover button reveals full path
        name_row = tk.Frame(nc, bg=SURFACE)
        name_row.pack(fill="x")
        tk.Label(name_row, text=short, bg=SURFACE, fg=TEXT,
                 font=self.F["body"], anchor="w").pack(side="left")
        if slot:
            tk.Label(name_row, text=f"   {slot}", bg=SURFACE, fg=MUTED,
                     font=self.F["small"]).pack(side="left", padx=(8, 0))

        badge = tk.Label(top, text="", bg=SURFACE, fg=MUTED,
                         font=self.F["small"], padx=6)
        badge.pack(side="right", anchor="n")

        br = tk.Frame(card, bg=SURFACE, padx=16, pady=8)
        br.pack(fill="x")
        for role in ROLES:
            tk.Button(br, text=role,
                      command=lambda p=port, r=role: self._manual(p, r),
                      bg=ROLE_SURF[role], fg=ROLE_FG[role],
                      font=self.F["small"], relief="flat",
                      padx=14, pady=6, cursor="hand2").pack(
                          side="left", padx=(0, 10))
        self.port_data[port] = {"type": "serial", "card": card, "badge": badge}

    def _make_camera_card(self, info):
        path   = info["path"]
        name   = info["name"]
        vendor = info.get("vendor", "")
        idx    = info["opencv_idx"]

        card = tk.Frame(self._portlist, bg=SURFACE,
                        highlightbackground=BORDER, highlightthickness=1)
        card.pack(fill="x", pady=5, ipady=2)

        top = tk.Frame(card, bg=SURFACE, padx=16, pady=10)
        top.pack(fill="x")
        nc = tk.Frame(top, bg=SURFACE)
        nc.pack(side="left", fill="x", expand=True)

        title_row = tk.Frame(nc, bg=SURFACE)
        title_row.pack(fill="x")
        tk.Label(title_row, text=name, bg=SURFACE, fg=TEXT,
                 font=self.F["body"], anchor="w").pack(side="left")
        if vendor:
            tk.Label(title_row, text=f"  ({vendor})",
                     bg=SURFACE, fg=MUTED,
                     font=self.F["small"]).pack(side="left")
        tk.Label(nc, text=os.path.basename(path),
                 bg=SURFACE, fg=MUTED, font=self.F["small"],
                 anchor="w").pack(fill="x")
        tk.Label(nc, text=f"lerobot: index_or_path: {idx}",
                 bg=SURFACE, fg=MUTED, font=self.F["small"],
                 anchor="w").pack(fill="x", pady=(2, 0))

        badge = tk.Label(top, text=f"  index {idx}  ",
                         bg=ROLE_SURF["Camera"], fg=ROLE_FG["Camera"],
                         font=self.F["small"], padx=4)
        badge.pack(side="right", anchor="n")

        br = tk.Frame(card, bg=SURFACE, padx=16, pady=8)
        br.pack(fill="x")
        tk.Button(br, text="Assign Camera",
                  command=lambda p=path: self._manual(p, "Camera"),
                  bg=ROLE_SURF["Camera"], fg=ROLE_FG["Camera"],
                  font=self.F["small"], relief="flat",
                  padx=14, pady=6, cursor="hand2").pack(side="left")
        tk.Button(br, text="Preview",
                  command=lambda i=idx, n=name, p=path:
                      self._show_camera_preview(i, n, p),
                  bg=SURFACE, fg=INFO,
                  font=self.F["small"], relief="flat",
                  padx=12, pady=6, cursor="hand2",
                  highlightbackground=BORDER,
                  highlightthickness=1).pack(side="left", padx=(10, 0))
        lerobot_arg = (f'front: {{type: opencv, index_or_path: {idx}, '
                       f'width: 640, height: 480, fps: 30}}')
        tk.Button(br, text="Copy arg",
                  command=lambda a=lerobot_arg: self._copy_to_clipboard(a),
                  bg=SURFACE, fg=MUTED,
                  font=self.F["small"], relief="flat",
                  padx=12, pady=6, cursor="hand2",
                  highlightbackground=BORDER,
                  highlightthickness=1).pack(side="left", padx=(10, 0))

        self.port_data[path] = {
            "type": "camera", "card": card, "badge": badge,
            "opencv_idx": idx}

    def _badge(self, port, role, method):
        if port not in self.port_data: return
        label = {"saved": "saved", "identified": "identified",
                 "manual": "manual"}.get(method, method)
        self.port_data[port]["badge"].configure(
            text=f"  {role}  -  {label}  ",
            bg=ROLE_SURF[role], fg=ROLE_FG[role])

    def _refresh_highlight(self, port, role):
        if port not in self.port_data: return
        tint = ROLE_SURF[role]
        card = self.port_data[port]["card"]
        card.configure(bg=tint, highlightbackground=ROLE_BORDER[role])
        for w in card.winfo_children():
            try: w.configure(bg=tint)
            except Exception: pass
            for ww in w.winfo_children():
                try: ww.configure(bg=tint)
                except Exception: pass

    # ── Assignments ───────────────────────────────────────────────────────────

    def _assign_port(self, port, role):
        sid = hw_serial(port)
        if sid:
            self.registry[sid] = role
            save_registry(self.registry)
        self.assignments[port] = role
        self._ui(self._badge, port, role, "identified")
        self._ui(self._refresh_highlight, port, role)
        self._ui(self._update_status, role, port)
        self._ui(self._check_ready)
        self._ui(self._generate_lerobot_cmd)

    def _apply_assignments(self, assignments):
        for port, role in assignments.items():
            self.assignments[port] = role
            self._refresh_highlight(port, role)
            self._update_status(role, port)
        self._check_ready()

    def _manual(self, port, role):
        for p in list(self.assignments):
            if self.assignments[p] == role and p != port:
                del self.assignments[p]
                if p in self.port_data:
                    c = self.port_data[p]["card"]
                    c.configure(bg=SURFACE, highlightbackground=BORDER)
                    for w in c.winfo_children():
                        try: w.configure(bg=SURFACE)
                        except Exception: pass
                        for ww in w.winfo_children():
                            try: ww.configure(bg=SURFACE)
                            except Exception: pass
                self._update_status(role, None)
        self._assign_port(port, role)

    # ── lerobot command panel ─────────────────────────────────────────────────


    def _build_launch_panel(self, parent):
        """Apply stable by-id ports to launch files — replaces manual nano editing."""
        lp = tk.Frame(parent, bg=SURFACE,
                      highlightbackground=BORDER, highlightthickness=1)
        lp.grid(row=2, column=0, sticky="ew", pady=(12, 0))
        lp.columnconfigure(0, weight=1)

        tk.Label(lp, text="Apply to Launch Files",
                 bg=SURFACE, fg=TEXT, font=self.F["h2"],
                 padx=16, pady=10, anchor="w").pack(fill="x")

        fields = tk.Frame(lp, bg=SURFACE, padx=16)
        fields.pack(fill="x")
        fields.columnconfigure(1, weight=1)

        for i, (label, var) in enumerate([
            ("Leader",   self._leader_launch),
            ("Follower", self._follower_launch),
        ]):
            tk.Label(fields, text=label,
                     bg=SURFACE, fg=ROLE_FG[label],
                     font=self.F["btn"], width=8,
                     anchor="w").grid(row=i, column=0, sticky="w", pady=3)
            tk.Entry(fields, textvariable=var,
                     bg=BG, fg=MUTED, font=self.F["small"],
                     relief="flat",
                     highlightbackground=BORDER,
                     highlightthickness=1,
                     insertbackground=TEXT).grid(
                         row=i, column=1, sticky="ew",
                         padx=(8, 0), ipady=4)

        br = tk.Frame(lp, bg=SURFACE, padx=16, pady=8)
        br.pack(fill="x")

        tk.Button(br, text="Apply",
                  command=self._apply_to_launch_files,
                  bg=TEXT, fg="#ffffff", font=self.F["small"],
                  relief="flat", padx=16, pady=7,
                  cursor="hand2").pack(side="left")

        self._apply_result_lbl = tk.Label(br, textvariable=self._apply_result,
                                          bg=SURFACE, fg=SUCCESS,
                                          font=self.F["small"])
        self._apply_result_lbl.pack(side="left", padx=12)
        tk.Frame(lp, height=6, bg=SURFACE).pack()

    def _apply_to_launch_files(self):
        import re as _re
        leader   = next((p for p, r in self.assignments.items()
                         if r == "Leader"), None)
        follower = next((p for p, r in self.assignments.items()
                         if r == "Follower"), None)

        def stable(port):
            if not port: return None
            if "/dev/serial/by-id/" in port: return port
            m = [p for p in glob.glob("/dev/serial/by-id/*")
                 if os.path.realpath(p) == os.path.realpath(port)]
            return m[0] if m else port

        def rewrite(launch_file, new_port):
            launch_file = os.path.expanduser(launch_file)
            if not os.path.exists(launch_file):
                return "notfound"
            with open(launch_file) as f:
                lines = f.readlines()
            new_lines = []
            found = False
            i = 0
            while i < len(lines):
                line = lines[i]
                # Match DeclareLaunchArgument port_name block:
                # looks for a line that is ONLY the port_name string
                # e.g. "            'port_name',"
                stripped = line.strip()
                if stripped in ("'port_name',", '"port_name",') and i + 1 < len(lines):
                    next_line = lines[i + 1]
                    if "default_value=" in next_line:
                        indent = len(next_line) - len(next_line.lstrip())
                        spaces = next_line[:indent]
                        new_next = f"{spaces}default_value='{new_port}',\n"
                        new_lines.append(line)
                        new_lines.append(new_next)
                        found = True
                        i += 2
                        continue
                new_lines.append(line)
                i += 1
            if not found:
                return "nomatch"
            with open(launch_file, 'w') as f:
                f.writelines(new_lines)
            return "ok"
        errors, results = [], []
        for port, path, label in [
            (leader,   self._leader_launch.get().strip(),   "Leader"),
            (follower, self._follower_launch.get().strip(), "Follower"),
        ]:
            path = os.path.expanduser(path)
            sp = stable(port)
            if not sp:
                errors.append(f"{label} not assigned")
                continue
            result = rewrite(path, sp)
            if result == "ok":
                results.append(label)
            elif result == "notfound":
                errors.append(f"{label}: file not found at {path}")
            else:
                errors.append(f"{label}: port_name declaration not found in file")

        # Also update omx_f.launch.py for MoveIt2 bringup
        sp_f = stable(follower)
        if sp_f:
            bringup = os.path.expanduser(
                self._follower_launch.get().strip().replace(
                "omx_f_follower_ai.launch.py", "omx_f.launch.py"))
            rewrite(bringup, sp_f)

        if errors:
            self._apply_result.set("Error: " + ", ".join(errors))
            self._apply_result_lbl.configure(fg=DANGER)
        else:
            self._apply_result.set("Applied: " + " + ".join(results))
            self._apply_result_lbl.configure(fg=SUCCESS)
            self.root.after(3000, lambda: self._apply_result.set(""))

    def _build_lerobot_panel(self):
        self._lr_panel = tk.Frame(self.root, bg=SURFACE,
                                  highlightbackground=BORDER,
                                  highlightthickness=1)
        self._lr_panel.pack(fill="x")
        self._lr_toggle = tk.Label(
            self._lr_panel,
            text="[+]  lerobot-record Command  (click to expand)",
            bg=SURFACE, fg=TEXT, font=self.F["h2"],
            padx=16, pady=10, anchor="w", cursor="hand2")
        self._lr_toggle.pack(fill="x")
        self._lr_toggle.bind("<Button-1>", self._toggle_lerobot)
        # Fixed-height scrollable container for lerobot body
        self._lr_scroll_frame = tk.Frame(self._lr_panel, bg=SURFACE,
                                         height=320)
        self._lr_scroll_frame.pack_propagate(False)

        lr_canvas = tk.Canvas(self._lr_scroll_frame, bg=SURFACE,
                              highlightthickness=0, height=320)
        lr_vsb = tk.Scrollbar(self._lr_scroll_frame, orient="vertical",
                              command=lr_canvas.yview)
        lr_canvas.configure(yscrollcommand=lr_vsb.set)
        lr_vsb.pack(side="right", fill="y")
        lr_canvas.pack(side="left", fill="both", expand=True)

        self._lr_body = tk.Frame(lr_canvas, bg=SURFACE)
        lr_canvas.create_window((0, 0), window=self._lr_body, anchor="nw")
        self._lr_body.bind("<Configure>",
            lambda e: lr_canvas.configure(
                scrollregion=lr_canvas.bbox("all")))
        lr_canvas.bind("<Configure>",
            lambda e: lr_canvas.itemconfigure(
                lr_canvas.find_all()[0], width=e.width))

        # Mouse wheel scrolling
        def _on_mousewheel(e):
            lr_canvas.yview_scroll(int(-1*(e.delta/120)), "units")
        lr_canvas.bind("<MouseWheel>", _on_mousewheel)
        lr_canvas.bind("<Button-4>",
            lambda e: lr_canvas.yview_scroll(-1, "units"))
        lr_canvas.bind("<Button-5>",
            lambda e: lr_canvas.yview_scroll(1, "units"))

        tk.Label(self._lr_body,
                 text="Note: use your HF account username "
                      "(run: hf auth whoami), not your token name.",
                 bg=SURFACE, fg=WARN,
                 font=self.F["small"], padx=16, pady=6,
                 anchor="w").pack(fill="x")

        fields = tk.Frame(self._lr_body, bg=SURFACE, padx=16, pady=4)
        fields.pack(fill="x")
        fields.columnconfigure(1, weight=1)
        fields.columnconfigure(3, weight=1)

        _fs = tkfont.Font(font=self.F["small"], size=11)
        _fm = tkfont.Font(font=self.F["mono"],  size=11)

        def row(label, var, r, c=0, width=None, choices=None):
            tk.Label(fields, text=label, bg=SURFACE, fg=MUTED,
                     font=_fs).grid(
                         row=r, column=c, sticky="w", pady=2)
            if choices:
                om = tk.OptionMenu(fields, var, *choices)
                om.configure(bg=BG, fg=TEXT, font=_fs,
                             relief="flat",
                             highlightbackground=BORDER,
                             highlightthickness=1)
                om.grid(row=r, column=c+1, sticky="ew",
                        padx=(6, 14 if c == 0 else 0), ipady=1)
            else:
                kw = {"width": width} if width else {}
                tk.Entry(fields, textvariable=var,
                         bg=BG, fg=TEXT, font=_fm,
                         relief="flat",
                         highlightbackground=BORDER,
                         highlightthickness=1,
                         insertbackground=TEXT, **kw).grid(
                             row=r, column=c+1, sticky="ew",
                             padx=(6, 14 if c == 0 else 0), ipady=3)

        row("HF User",    self._hf_user_var,  0, c=0)
        row("Dataset",    self._dataset_var,   0, c=2)
        row("Episodes",   self._episodes_var,  1, c=0, width=6)
        row("Task",       self._task_var,      1, c=2)
        row("Camera idx", self._cam_idx_var,   2, c=0, width=6)
        row("Display",    self._display_var,   2, c=2,
            choices=["true", "false"])

        tk.Frame(self._lr_body, height=8, bg=SURFACE).pack()

        cf = tk.Frame(self._lr_body, bg=SURFACE, padx=16)
        cf.pack(fill="x")
        vsb = tk.Scrollbar(cf, orient="vertical")
        self._cmd_text = tk.Text(
            cf, height=11, bg="#0f1117", fg="#a6e3a1",
            font=self.F["mono"], relief="flat",
            wrap="none", state="disabled",
            yscrollcommand=vsb.set)
        xsb = tk.Scrollbar(cf, orient="horizontal",
                           command=self._cmd_text.xview)
        self._cmd_text.configure(xscrollcommand=xsb.set)
        vsb.configure(command=self._cmd_text.yview)
        vsb.pack(side="right", fill="y")
        self._cmd_text.pack(fill="x", expand=True)
        xsb.pack(fill="x")

        br = tk.Frame(self._lr_body, bg=SURFACE, padx=16, pady=10)
        br.pack(fill="x")
        tk.Button(br, text="Generate",
                  command=self._generate_lerobot_cmd,
                  bg=TEXT, fg="#ffffff", font=self.F["small"],
                  relief="flat", padx=16, pady=8,
                  cursor="hand2").pack(side="left", padx=(0, 10))
        tk.Button(br, text="Copy",
                  command=self._copy_lerobot_cmd,
                  bg=SURFACE, fg=INFO, font=self.F["small"],
                  relief="flat", padx=16, pady=8,
                  cursor="hand2",
                  highlightbackground=BORDER,
                  highlightthickness=1).pack(side="left")
        self._cmd_status = tk.Label(br, text="",
                                    bg=SURFACE, fg=SUCCESS,
                                    font=self.F["small"])
        self._cmd_status.pack(side="left", padx=14)
        tk.Frame(self._lr_body, height=8, bg=SURFACE).pack()

        for var in [self._hf_user_var, self._dataset_var,
                    self._cam_idx_var, self._episodes_var,
                    self._task_var, self._display_var]:
            var.trace_add("write",
                lambda *_: self._generate_lerobot_cmd())

    def _toggle_lerobot(self, event=None):
        if self._lerobot_expanded:
            self._lr_scroll_frame.pack_forget()
            self._lr_toggle.configure(
                text="[+]  lerobot-record Command  (click to expand)")
            self._lerobot_expanded = False
        else:
            self._lr_scroll_frame.pack(fill="x")
            self._lr_toggle.configure(
                text="[-]  lerobot-record Command  (click to collapse)")
            self._lerobot_expanded = True
            self._generate_lerobot_cmd()

    def _generate_lerobot_cmd(self):
        leader   = next((p for p, r in self.assignments.items()
                         if r == "Leader"), None)
        follower = next((p for p, r in self.assignments.items()
                         if r == "Follower"), None)
        camera   = next((p for p, r in self.assignments.items()
                         if r == "Camera"), None)

        hf_user  = self._hf_user_var.get().strip()
        dataset  = self._dataset_var.get().strip()
        episodes = self._episodes_var.get().strip() or "5"
        task     = self._task_var.get().strip() or "Pick up the object"
        display  = self._display_var.get().strip() or "true"
        cam_idx  = self._cam_idx_var.get().strip()

        if camera and camera in self.port_data:
            idx = self.port_data[camera].get("opencv_idx")
            if idx is not None:
                cam_idx = str(idx)
                self._cam_idx_var.set(cam_idx)

        def stable(port):
            if not port: return "<not assigned>"
            if "/dev/serial/by-id/" in port: return port
            m = [p for p in glob.glob("/dev/serial/by-id/*")
                 if os.path.realpath(p) == os.path.realpath(port)]
            return m[0] if m else port

        repo_id = (f"{hf_user}/{dataset}" if hf_user
                   else f"<HF_USER>/{dataset}")
        cmd = (
            f"# Prerequisites\n"
            f"docker stop open_manipulator\n"
            f"source ~/miniconda3/bin/activate && conda activate lerobot\n\n"
            f"lerobot-record \\\n"
            f"  --robot.type=omx_follower \\\n"
            f"  --robot.port={stable(follower)} \\\n"
            f"  --robot.id=omx_follower_arm \\\n"
            f'  --robot.cameras="{{front: {{type: opencv, '
            f'index_or_path: {cam_idx}, '
            f'width: 640, height: 480, fps: 30}}}}" \\\n'
            f"  --teleop.type=omx_leader \\\n"
            f"  --teleop.port={stable(leader)} \\\n"
            f"  --teleop.id=omx_leader_arm \\\n"
            f"  --display_data={display} \\\n"
            f"  --dataset.repo_id={repo_id} \\\n"
            f"  --dataset.num_episodes={episodes} \\\n"
            f'  --dataset.single_task="{task}"'
        )
        self._cmd_text.configure(state="normal")
        self._cmd_text.delete("1.0", "end")
        self._cmd_text.insert("end", cmd)
        self._cmd_text.configure(state="disabled")

    def _copy_lerobot_cmd(self):
        cmd = self._cmd_text.get("1.0", "end").strip()
        if not cmd:
            self._generate_lerobot_cmd()
            cmd = self._cmd_text.get("1.0", "end").strip()
        self.root.clipboard_clear()
        self.root.clipboard_append(cmd)
        self._cmd_status.configure(text="Copied!", fg=SUCCESS)
        self.root.after(2000,
            lambda: self._cmd_status.configure(text=""))

    def _copy_to_clipboard(self, text):
        self.root.clipboard_clear()
        self.root.clipboard_append(text)

    # ── Camera preview ────────────────────────────────────────────────────────

    def _show_camera_preview(self, opencv_idx, cam_name, dev_path):
        # Script receives dev_path, idx, cam_name as CLI args —
        # avoids f-string injection if cam_name contains quotes.
        script = """
import cv2, time, sys
dev_path = sys.argv[1]
idx      = int(sys.argv[2])
cam_name = sys.argv[3]
cap = None
for attempt in [
    lambda: cv2.VideoCapture(idx, cv2.CAP_V4L2),
    lambda: cv2.VideoCapture(idx),
    lambda: cv2.VideoCapture(dev_path, cv2.CAP_V4L2),
    lambda: cv2.VideoCapture(dev_path),
]:
    try:
        c = attempt()
        if c.isOpened():
            cap = c
            break
        c.release()
    except Exception:
        pass
if cap is None or not cap.isOpened():
    print("Cannot open camera index " + str(idx), file=sys.stderr)
    sys.exit(1)
cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
cap.set(cv2.CAP_PROP_FRAME_WIDTH,  640)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
time.sleep(0.8)
for _ in range(10): cap.read()
title = f"{cam_name} (index {idx}) - press Q to close"
while True:
    ret, frame = cap.read()
    if not ret or frame is None:
        time.sleep(0.03)
        continue
    cv2.imshow(title, frame)
    if cv2.waitKey(30) & 0xFF == ord("q"):
        break
cap.release()
cv2.destroyAllWindows()
"""
        # Check cv2 is importable before launching subprocess
        try:
            import importlib
            importlib.import_module("cv2")
        except ImportError:
            messagebox.showinfo(
                "OpenCV not installed",
                "Install OpenCV to use camera preview:\n\n"
                "pip install opencv-python --break-system-packages\n\n"
                "Then re-run the tool.")
            return

        try:
            subprocess.Popen([sys.executable, "-c", script,
                              dev_path, str(opencv_idx), cam_name])
        except Exception as e:
            messagebox.showerror("Preview error", str(e))

    # ── Misc ──────────────────────────────────────────────────────────────────

    def _rescan(self):
        self._stop_watch = True
        self.root.after(300, self._do_rescan)

    def _do_rescan(self):
        self._stop_watch = False
        self._stop_pulse()
        for role in ("Leader", "Follower", "Camera"):
            self._update_status(role, None)
        self._ready_card.grid_forget()
        threading.Thread(target=self._run_detection, daemon=True).start()

    def _forget(self):
        if not messagebox.askyesno(
                "Forget Boards",
                "Clear the saved board registry?\n\n"
                "You will need to re-identify your arms next time."):
            return
        self.registry = {}
        save_registry({})
        self.assignments.clear()
        for role in ("Leader", "Follower", "Camera"):
            self._update_status(role, None)
        self._ready_card.grid_forget()
        self._rescan()

    def _ui(self, fn, *args, **kwargs):
        self.root.after(0, lambda: fn(*args, **kwargs))


if __name__ == "__main__":
    root = tk.Tk()
    root.geometry("1200x820")

    # Enable Xft antialiasing for tkinter text rendering.
    # Xft is the X FreeType font renderer — enabling it gives
    # smooth subpixel-antialiased text instead of bitmap rendering.
    try:
        root.tk.call('tk', 'scaling', root.winfo_fpixels('1i') / 72.0)
    except Exception:
        pass



    App(root)
    root.mainloop()