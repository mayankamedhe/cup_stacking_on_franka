"""Live Franka hardware dashboard. Run in your ROS1 / FrankaPy environment.

    python franka_dashboard.py
    python franka_dashboard.py --state-topic /franka_state_controller/franka_states
"""
import argparse
from collections import deque
import queue
import signal
import time
import tkinter as tk
from tkinter import filedialog, messagebox, ttk

import numpy as np
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from matplotlib.figure import Figure

import config as C
from franka_dashboard_backend import (
    Controller, FrankaBackend, GROUPS, export_csv, rotation_from_rpy, rpy_from_rotation,
)


class Dashboard:
    def __init__(self, root, controller, history_seconds=60):
        self.root, self.controller = root, controller
        self.history = deque(maxlen=int(history_seconds*controller.rate))
        self.history_seconds = history_seconds
        self.last_draw = 0
        self.closing = False
        self.loaded = False
        self.command_buttons = []
        root.title("Franka · Force & Motion")
        root.geometry("1450x920")
        root.minsize(1100, 750)
        root.configure(bg="#101923")
        style = ttk.Style(root)
        style.theme_use("clam")
        style.configure("TFrame", background="#101923")
        style.configure("TLabel", background="#101923", foreground="#d9e5ef")
        style.configure("TLabelframe", background="#101923", foreground="#d9e5ef")
        style.configure("TLabelframe.Label", background="#101923", foreground="#89bbd7")
        style.configure("TNotebook", background="#101923")
        style.configure("TButton", padding=(9, 5))
        style.configure("Header.TLabel", font=("TkDefaultFont", 18, "bold"))
        style.configure("Readout.TLabel", font=("TkFixedFont", 10))

        header = ttk.Frame(root, padding=12)
        header.pack(fill="x")
        ttk.Label(header, text="FRANKA  /  FORCE & MOTION", style="Header.TLabel").pack(side="left")
        self.enable_button = ttk.Button(header, text="Enable controls", command=self.toggle_enable)
        self.enable_button.pack(side="right", padx=12)
        tk.Button(header, text="STOP ARM + GRIPPER  [Esc]", command=self.stop,
                  bg="#d63e4a", fg="white", activebackground="#b92532",
                  font=("TkDefaultFont", 12, "bold"), relief="flat", padx=16, pady=8).pack(side="right")
        self.connection = tk.StringVar(value="Waiting for robot telemetry…")
        ttk.Label(root, textvariable=self.connection, padding=(14, 0, 14, 5)).pack(anchor="w")
        ttk.Label(root, text="External wrench is an arm estimate. Measured gripping force: unavailable. "
                  "Software STOP depends on ROS; keep the physical stop accessible.",
                  foreground="#e8ba76", padding=(14, 0, 14, 8)).pack(anchor="w")

        body = ttk.Panedwindow(root, orient="horizontal")
        body.pack(fill="both", expand=True, padx=12)
        controls = ttk.Frame(body, width=415)
        plots = ttk.Frame(body)
        body.add(controls, weight=0)
        body.add(plots, weight=1)

        self.live = tk.StringVar(value="No state yet")
        ttk.Label(controls, textvariable=self.live, style="Readout.TLabel", padding=8).pack(fill="x")
        tabs = ttk.Notebook(controls)
        tabs.pack(fill="both", expand=True)
        pose_tab, joint_tab, grip_tab, force_tab = [ttk.Frame(tabs, padding=12) for _ in range(4)]
        for tab, label in zip((pose_tab, joint_tab, grip_tab, force_tab),
                              ("Cartesian", "Joints", "Gripper", "Force")):
            tabs.add(tab, text=label)
        self._build_pose(pose_tab)
        self._build_joints(joint_tab)
        self._build_gripper(grip_tab)
        self._build_force(force_tab)

        plot_toolbar = ttk.Frame(plots)
        plot_toolbar.pack(fill="x", pady=(0, 5))
        self.paused = tk.BooleanVar(value=False)
        ttk.Checkbutton(plot_toolbar, text="Freeze plots", variable=self.paused).pack(side="left")
        ttk.Button(plot_toolbar, text="Export history CSV", command=self.export).pack(side="right")
        ttk.Label(plot_toolbar, text=f"Rolling {history_seconds:g}s · raw values").pack(side="right", padx=12)
        self.plot_selectors = []
        self.axes = []
        self.lines = []
        self.canvases = []
        grid = ttk.Frame(plots)
        grid.pack(fill="both", expand=True)
        names = [details[0] for details in GROUPS.values()]
        for i, initial in enumerate(("force_base", "torque_base", "joint_torque", "joint_external")):
            cell = ttk.Frame(grid)
            cell.grid(row=i//2, column=i%2, sticky="nsew", padx=4, pady=4)
            grid.rowconfigure(i//2, weight=1)
            grid.columnconfigure(i%2, weight=1)
            var = tk.StringVar(value=GROUPS[initial][0])
            choice = ttk.Combobox(cell, textvariable=var, values=names, state="readonly", width=34)
            choice.pack(fill="x")
            choice.bind("<<ComboboxSelected>>", lambda event, index=i: self.configure_plot(index))
            figure = Figure(figsize=(4, 2.7), dpi=95, facecolor="#101923")
            figure.subplots_adjust(left=0.16, right=0.97, top=0.91, bottom=0.20)
            ax = figure.add_subplot(111, facecolor="#162431")
            canvas = FigureCanvasTkAgg(figure, master=cell)
            canvas.get_tk_widget().pack(fill="both", expand=True)
            self.plot_selectors.append(var)
            self.axes.append(ax)
            self.canvases.append(canvas)
            self.lines.append([])
            self.configure_plot(i)
        self.values_text = tk.StringVar()
        ttk.Label(plots, textvariable=self.values_text, style="Readout.TLabel", padding=6).pack(fill="x")
        self.log = tk.Text(root, height=4, bg="#162431", fg="#bcd0df", relief="flat", state="disabled")
        self.log.pack(fill="x", padx=12, pady=8)
        self.write_log("Hardware mode. Review GUI_* limits in config.py before enabling controls.")
        self.write_log(controller.backend.topic_note)
        if not controller.backend.has_gripper:
            self.write_log("Gripper actions unavailable. Arm telemetry remains available.")
        root.bind("<Escape>", lambda event: self.stop())
        root.protocol("WM_DELETE_WINDOW", self.close)
        root.after(50, self.tick)

    def fields(self, parent, labels, values):
        variables = []
        for label, value in zip(labels, values):
            row = ttk.Frame(parent)
            row.pack(fill="x", pady=3)
            ttk.Label(row, text=label, width=24).pack(side="left")
            var = tk.StringVar(value=str(value))
            ttk.Entry(row, textvariable=var, width=12).pack(side="right")
            variables.append(var)
        return variables

    def button(self, parent, label, command):
        button = ttk.Button(parent, text=label, command=lambda: self.attempt(command))
        button.pack(fill="x", pady=5)
        self.command_buttons.append(button)
        return button

    def note(self, parent, text):
        ttk.Label(parent, text=text, wraplength=345, foreground="#9eb1c1").pack(fill="x", pady=8)

    def _build_pose(self, parent):
        self.note(parent, "Tool pose in robot base/world frame. Orientation: Rz(yaw) Ry(pitch) Rx(roll).")
        self.pose_fields = self.fields(parent, ("X (m)", "Y (m)", "Z (m)", "Roll (deg)",
                                               "Pitch (deg)", "Yaw (deg)", "Duration (s)"),
                                       (0.4, 0, 0.3, 180, 0, 0, 3))
        ttk.Button(parent, text="Copy measured pose", command=lambda: self.attempt(self.load_pose)).pack(fill="x", pady=5)
        self.button(parent, "Move to pose", self.send_pose)
        self.jog_mm = self.fields(parent, ("Jog step (mm)",), (5,))[0]
        jog = ttk.Frame(parent)
        jog.pack(fill="x", pady=8)
        for axis in range(3):
            for sign in (-1, 1):
                button = ttk.Button(jog, text="XYZ"[axis] + (" −" if sign < 0 else " +"),
                                    command=lambda a=axis, s=sign: self.attempt(lambda: self.jog(a, s)))
                button.grid(row=axis, column=0 if sign < 0 else 1, sticky="ew", padx=4, pady=3)
                self.command_buttons.append(button)
        jog.columnconfigure((0, 1), weight=1)
        self.note(parent, f"Max move: {C.GUI_MAX_TRANSLATION*1000:g} mm / {C.GUI_MAX_ROTATION_DEG:g}°. "
                  f"XYZ limits: {self.controller.limits.xyz_min} to {self.controller.limits.xyz_max} m.")

    def _build_joints(self, parent):
        self.note(parent, "Joint targets in degrees. Copy current positions before editing.")
        self.joint_fields = self.fields(parent, [f"Joint {i} (deg)" for i in range(1, 8)] + ["Duration (s)"],
                                        [0, -45, 0, -135, 0, 90, 45, 4])
        ttk.Button(parent, text="Copy measured joints", command=lambda: self.attempt(self.load_joints)).pack(fill="x", pady=5)
        self.button(parent, "Move joints", self.send_joints)
        self.note(parent, f"Max step: {C.GUI_MAX_JOINT_STEP_DEG:g}° per joint. "
                  "Joint motions use FrankaPy joint limits and collision checks. "
                  "The GUI Cartesian box does not constrain a joint trajectory.")

    def _build_gripper(self, parent):
        self.note(parent, "Grasp force is a command to the Franka hand. The hand provides no measured "
                  "finger/contact force signal. Arm external force is a separate estimate.")
        self.grip_fields = self.fields(parent, ("Target width (mm)", "Speed (mm/s)",
                                               "Grasp force (N)", "Width tolerance ± (mm)"),
                                       (C.GRASP_WIDTH*1000, 20, C.GRASP_FORCE, 1))
        self.button(parent, "Move to width", lambda: self.send_gripper(False))
        self.button(parent, "Grasp at width + force", lambda: self.send_gripper(True))
        self.button(parent, "Open gripper (80 mm)", lambda: self.send_gripper(False, 80))
        self.note(parent, "Move/open uses the position controller; the force field applies to grasp. "
                  "Grasp succeeds only within the selected width tolerance. "
                  "A successful action does not prove a cup is securely held.")
        self.note(parent, f"Grasp command range: 0.1–{C.GUI_MAX_GRASP_FORCE:g} N. "
                  "The installed hand may reject values below its supported range.")

    def _build_force(self, parent):
        self.note(parent, "Bounded Cartesian wrench pulse in robot base axes. "
                  "Uses FrankaPy apply_effector_forces_torques; this is a commanded wrench, "
                  "not a closed-loop contact-force setpoint.")
        self.force_fields = self.fields(parent, ("Fx (N)", "Fy (N)", "Fz (N)", "Tx (Nm)",
                                                "Ty (Nm)", "Tz (Nm)", "Duration (s)",
                                                "Ramp up/down (s)", "Travel cap (mm)", "Rotation cap (deg)"),
                                        (0, 0, 0, 0, 0, 0, 1, 0.25, 5, 2))
        self.button(parent, "Apply force / torque pulse", self.send_force)
        self.note(parent, f"Force norm ≤ {C.GUI_MAX_FORCE:g} N; torque norm ≤ {C.GUI_MAX_TORQUE:g} Nm. "
                  "Travel caps turn off the wrench; inertia can carry the arm farther. "
                  "A pulse may move unconstrained axes.")

    @staticmethod
    def numbers(fields):
        values = [float(var.get()) for var in fields]
        if not np.all(np.isfinite(values)):
            raise ValueError("All entries must be finite numbers")
        return values

    def attempt(self, function):
        try:
            function()
        except Exception as exc:
            self.write_log(str(exc))
            messagebox.showerror("Command not sent", str(exc), parent=self.root)

    def load_pose(self):
        s = self.controller.latest
        if not self.controller.fresh():
            raise ValueError("Wait for fresh robot state")
        for var, value in zip(self.pose_fields, [*s.xyz, *rpy_from_rotation(s.rotation)]):
            var.set(f"{value:.6f}")

    def load_joints(self):
        if not self.controller.fresh():
            raise ValueError("Wait for fresh robot state")
        for var, value in zip(self.joint_fields, np.rad2deg(self.controller.latest.joints)):
            var.set(f"{value:.6f}")

    def send_pose(self):
        v = self.numbers(self.pose_fields)
        self.controller.submit("pose", dict(xyz=v[:3], rotation=rotation_from_rpy(v[3:6]), duration=v[6]))

    def jog(self, axis, sign):
        step = self.numbers([self.jog_mm])[0]
        if not 0 < step <= C.GUI_MAX_TRANSLATION*1000:
            raise ValueError("Jog step must be positive and within the move limit")
        delta = np.zeros(3)
        delta[axis] = sign*step/1000
        self.controller.submit("jog", dict(delta=delta, duration=self.numbers(self.pose_fields[-1:])[0]))

    def send_joints(self):
        v = self.numbers(self.joint_fields)
        self.controller.submit("joints", dict(joints=np.deg2rad(v[:7]), duration=v[7]))

    def send_gripper(self, grasp, width=None):
        v = self.numbers(self.grip_fields)
        self.controller.submit("gripper", dict(width=(v[0] if width is None else width)/1000,
                               speed=v[1]/1000, force=v[2], epsilon=v[3]/1000, grasp=grasp))

    def send_force(self):
        v = self.numbers(self.force_fields)
        self.controller.submit("force", dict(forces=v[:3], torques=v[3:6], duration=v[6], ramp=v[7],
                                             translation=v[8]/1000, rotation=np.deg2rad(v[9])))

    def toggle_enable(self):
        if self.controller.armed:
            self.stop()
        else:
            self.attempt(self.controller.arm)

    def stop(self):
        self.controller.stop()

    def selected_key(self, i):
        return next(key for key, detail in GROUPS.items() if detail[0] == self.plot_selectors[i].get())

    def configure_plot(self, index):
        key = self.selected_key(index)
        _, unit, labels = GROUPS[key]
        ax = self.axes[index]
        ax.clear()
        ax.tick_params(colors="#a5bacb", labelsize=8)
        for spine in ax.spines.values():
            spine.set_color("#345064")
        ax.set_xlabel("seconds ago", color="#a5bacb", fontsize=9)
        ax.set_ylabel(unit, color="#a5bacb", fontsize=9)
        ax.set_xlim(-self.history_seconds, 0)
        ax.grid(alpha=0.15)
        colors = ("#63d6cc", "#edbd70", "#a89aff", "#6db8ee", "#ee8caa", "#a5cb75", "#e89364")
        self.lines[index] = [ax.plot([], [], label=label, color=colors[i], linewidth=1.2)[0]
                             for i, label in enumerate(labels)]
        ax.legend(loc="upper left", ncol=min(len(labels), 4), fontsize=7,
                  facecolor="#162431", edgecolor="#345064", labelcolor="#d9e5ef")
        self.canvases[index].draw_idle()
        self.last_draw = 0

    def draw_plots(self):
        if not self.history:
            return
        samples = list(self.history)
        now = time.monotonic()
        x = np.array([s.received-now for s in samples])
        readouts = []
        for i, (ax, lines) in enumerate(zip(self.axes, self.lines)):
            key = self.selected_key(i)
            _, unit, labels = GROUPS[key]
            data = np.array([s.values[key] for s in samples])
            for j, line in enumerate(lines):
                line.set_data(x, data[:, j])
            ax.relim()
            ax.autoscale_view(scalex=False)
            ax.set_xlim(-self.history_seconds, 0)
            latest = data[-1] if self.controller.fresh() else np.full(len(labels), np.nan)
            text = "  ".join(f"{label} {v:+.2f}" if np.isfinite(v) else f"{label} —"
                             for label, v in zip(labels, latest))
            readouts.append(f"{i+1}. {text} {unit}")
            self.canvases[i].draw_idle()
        self.values_text.set("\n".join(readouts))

    def export(self):
        samples = list(self.history)
        if not samples:
            return
        path = filedialog.asksaveasfilename(parent=self.root, defaultextension=".csv",
                                           initialfile="franka_forces.csv", filetypes=[("CSV", "*.csv")])
        if path:
            def save():
                export_csv(path, samples)
                self.write_log(f"Exported {len(samples)} samples to {path}")
            self.attempt(save)

    def write_log(self, text):
        self.log.configure(state="normal")
        self.log.insert("end", time.strftime("%H:%M:%S") + "  " + text + "\n")
        if int(self.log.index("end-1c").split(".")[0]) > 200:
            self.log.delete("1.0", "2.0")
        self.log.see("end")
        self.log.configure(state="disabled")

    def tick(self):
        try:
            while True:
                self.history.append(self.controller.samples.get_nowait())
        except queue.Empty:
            pass
        try:
            while True:
                self.write_log(self.controller.events.get_nowait())
        except queue.Empty:
            pass
        c = self.controller
        fresh = c.fresh()
        enabled = c.armed and not c.busy and not c.stopping and fresh and not self.closing
        for button in self.command_buttons:
            button.configure(state="normal" if enabled else "disabled")
        self.enable_button.configure(text="Disable controls" if c.armed else "Enable controls",
                                     state="normal" if fresh and not c.busy and not c.stopping and not self.closing else "disabled")
        if c.latest is not None:
            s = c.latest
            age = time.monotonic()-s.received
            width = s.values["gripper_width"][0]
            self.connection.set(f"{'LIVE' if fresh else 'STALE'} · state age {age:.2f}s · "
                                f"{'ENABLED' if c.armed else 'DISABLED'} · {'BUSY' if c.busy else 'IDLE'}"
                                + (f" · {c.error}" if c.error else ""))
            xyz = " ".join(f"{v:+.4f}" for v in s.xyz)
            joints = " ".join(f"{v:+.1f}" for v in np.rad2deg(s.joints))
            width_text = f"{width:.1f} mm" if np.isfinite(width) and fresh else "unavailable"
            self.live.set(f"XYZ (m)  {xyz}\nJ°  {joints}\nGripper width: {width_text}\nGripping force: not measured")
            if fresh and not self.loaded:
                self.load_pose()
                self.load_joints()
                self.loaded = True
        elif c.error:
            self.connection.set("No telemetry: " + c.error)
        if not self.paused.get() and time.monotonic()-self.last_draw > 0.2:
            self.draw_plots()
            self.last_draw = time.monotonic()
        if self.closing and not c.stopping and c.stop_failed:
            self.closing = False
            self.write_log("Close cancelled because STOP failed. Check the physical robot and retry STOP.")
        if self.closing and not c.stopping:
            self.root.destroy()
            return
        self.root.after(50, self.tick)

    def close(self):
        if self.closing:
            return
        self.closing = True
        self.controller.close()
        self.connection.set("Closing: sending arm and gripper cancellation…")


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--robot-num", type=int, default=1)
    parser.add_argument("--state-topic", help="Optional franka_msgs/FrankaState topic for this arm")
    parser.add_argument("--gripper-namespace", help="Default: /franka_gripper_<robot-num>")
    parser.add_argument("--rate", type=float, default=20, help="Telemetry sample rate, Hz (1–100)")
    parser.add_argument("--history", type=float, default=60, help="Rolling plot/export history, seconds (5–600)")
    args = parser.parse_args()
    if not 1 <= args.rate <= 100 or not 5 <= args.history <= 600 or args.robot_num < 1:
        parser.error("Require rate 1–100 Hz, history 5–600 s, and positive robot number")
    print("Connecting to FrankaPy / ROS1 (no motion on startup)…", flush=True)
    try:
        backend = FrankaBackend(args.robot_num, args.state_topic, args.gripper_namespace)
    except (ImportError, RuntimeError) as exc:
        raise SystemExit(f"Cannot connect: {exc}\nRun from the robot's ROS1/FrankaPy environment. "
                         "See README.md for dashboard setup.") from exc
    controller = Controller(backend, args.rate)
    root = tk.Tk()
    dashboard = Dashboard(root, controller, args.history)
    # Replace FrankaPy's exit-only handler so Ctrl+C also cancels the gripper.
    signal.signal(signal.SIGINT, lambda *_: root.after(0, dashboard.close))
    signal.signal(signal.SIGTERM, lambda *_: root.after(0, dashboard.close))
    controller.start()
    try:
        root.mainloop()
    finally:
        if not dashboard.closing:
            controller.close()


if __name__ == "__main__":
    main()
