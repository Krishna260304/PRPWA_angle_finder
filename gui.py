import csv
import math
import os
import queue
import threading
import time
from typing import Optional, Tuple, List, Dict, Any

import cv2
import numpy as np
from PIL import Image, ImageTk
import tkinter as tk
from tkinter import ttk, messagebox, filedialog

import config
from config import AppState
from camera import CameraManager
from vision import VisionTracker
from angle_tracker import AngleTracker
from serial_controller import SerialController


class StepperAngleApp:

    def __init__(self, root: tk.Tk):
        self.root = root
        self.root.title("Stepper Motor Angle Measurement Dashboard")
        self.root.geometry("1480x900")
        self.root.minsize(1240, 760)

        self.camera_mgr = CameraManager()
        self.vision_tracker = VisionTracker()
        self.serial_ctrl = SerialController()

        if self.camera_mgr.synthetic_cam:
            self.serial_ctrl.set_synthetic_camera(self.camera_mgr.synthetic_cam)

        self.state: AppState = AppState.SET_ORIGIN

        self.mouse_pos: Optional[Tuple[int, int]] = None
        self.is_mouse_over_feed: bool = False
        self.show_crosshair: bool = True

        self.measurements: List[Dict[str, Any]] = []

        self.disp_scale_x: float = 1.0
        self.disp_scale_y: float = 1.0
        self.disp_offset_x: int = 0
        self.disp_offset_y: int = 0
        self.last_frame_dims: Tuple[int, int] = (config.CAMERA_WIDTH, config.CAMERA_HEIGHT)

        self.event_queue: queue.Queue = queue.Queue()

        self.photo_image: Optional[ImageTk.PhotoImage] = None

        self._setup_styles()
        self._build_gui()
        self._setup_serial_callbacks()

        self._start_camera_initial()

        self.root.protocol("WM_DELETE_WINDOW", self.on_closing)

        self.root.after(120, self._set_initial_sash_position)

        self.root.after(config.FRAME_UPDATE_INTERVAL_MS, self._update_loop)

    def _setup_styles(self) -> None:
        self.style = ttk.Style()
        try:
            self.style.theme_use("clam")
        except Exception:
            pass

        self.bg_dark = "#0f172a"
        self.bg_panel = "#1e293b"
        self.bg_card = "#334155"
        self.text_primary = "#f8fafc"
        self.text_secondary = "#94a3b8"
        self.accent_blue = "#38bdf8"
        self.accent_green = "#22c55e"
        self.accent_amber = "#f59e0b"
        self.accent_red = "#ef4444"

        self.root.configure(bg=self.bg_dark)

        self.style.configure(".", background=self.bg_dark, foreground=self.text_primary,
                             font=("Segoe UI", 9))
        self.style.configure("TLabel", background=self.bg_dark, foreground=self.text_primary)
        self.style.configure("Card.TLabel", background=self.bg_panel, foreground=self.text_primary)
        self.style.configure("Header.TLabel", background=self.bg_dark, foreground=self.accent_blue,
                             font=("Segoe UI", 13, "bold"))
        self.style.configure("Instruction.TLabel", background=self.bg_dark, foreground=self.accent_amber,
                             font=("Segoe UI", 10, "bold"))

        self.style.configure("TFrame", background=self.bg_dark)
        self.style.configure("Panel.TFrame", background=self.bg_panel)
        self.style.configure("Card.TFrame", background=self.bg_panel, relief="solid", borderwidth=1)

        self.style.configure("TLabelframe", background=self.bg_panel, foreground=self.accent_blue)
        self.style.configure("TLabelframe.Label", background=self.bg_panel, foreground=self.accent_blue,
                             font=("Segoe UI", 9, "bold"))

        self.style.configure("TButton", font=("Segoe UI", 9, "bold"), padding=5)
        self.style.configure("Accent.TButton", font=("Segoe UI", 9, "bold"), padding=6)
        self.style.configure("Success.TButton", font=("Segoe UI", 10, "bold"), padding=8)
        self.style.configure("Danger.TButton", font=("Segoe UI", 9, "bold"), padding=5)
        self.style.configure("Warning.TButton", font=("Segoe UI", 9, "bold"), padding=5)

        self.style.configure("Treeview",
                             background="#1e293b",
                             foreground="#f8fafc",
                             fieldbackground="#1e293b",
                             rowheight=26,
                             font=("Segoe UI", 9))
        self.style.configure("Treeview.Heading",
                             background="#334155",
                             foreground="#38bdf8",
                             font=("Segoe UI", 9, "bold"))
        self.style.map("Treeview",
                       background=[("selected", "#0284c7")],
                       foreground=[("selected", "#ffffff")])

    def _build_gui(self) -> None:
        main_box = ttk.Frame(self.root, padding=8)
        main_box.pack(fill=tk.BOTH, expand=True)

        header_frame = ttk.Frame(main_box)
        header_frame.pack(fill=tk.X, pady=(0, 6))

        lbl_title = ttk.Label(header_frame, text="STEPPER MOTOR ANGLE MEASUREMENT DASHBOARD",
                              style="Header.TLabel")
        lbl_title.pack(side=tk.LEFT)

        self.lbl_instruction = ttk.Label(
            header_frame,
            text="STEP 1 OF 2: Click on the motor shaft center (Origin O) with the crosshair.",
            style="Instruction.TLabel"
        )
        self.lbl_instruction.pack(side=tk.RIGHT, padx=10)

        self.content_paned = ttk.PanedWindow(main_box, orient=tk.HORIZONTAL)
        self.content_paned.pack(fill=tk.BOTH, expand=True)

        left_frame = ttk.Frame(self.content_paned, style="Panel.TFrame", padding=6)
        self.content_paned.add(left_frame, weight=7)
        self._build_camera_view(left_frame)

        right_frame = ttk.Frame(self.content_paned, style="Panel.TFrame", padding=6)
        self.content_paned.add(right_frame, weight=3)
        self._build_control_panel(right_frame)

        self._build_status_bar(main_box)

    def _build_camera_view(self, parent: ttk.Frame) -> None:
        canvas_card = ttk.Frame(parent, style="Card.TFrame")
        canvas_card.pack(fill=tk.BOTH, expand=True)

        self.video_canvas = tk.Canvas(canvas_card, bg="#050811", highlightthickness=0,
                                      cursor="crosshair")
        self.video_canvas.pack(fill=tk.BOTH, expand=True, padx=2, pady=2)

        self.video_canvas.bind("<Motion>", self._on_canvas_mouse_move)
        self.video_canvas.bind("<Leave>", self._on_canvas_mouse_leave)
        self.video_canvas.bind("<Button-1>", self._on_canvas_click)

        cam_bar = ttk.Frame(parent, style="Panel.TFrame", padding=(2, 6, 2, 2))
        cam_bar.pack(fill=tk.X)

        ttk.Label(cam_bar, text="Camera:", style="Card.TLabel").pack(side=tk.LEFT, padx=(0, 4))
        self.cam_combo = ttk.Combobox(cam_bar, values=["Simulation Rig", "Camera 0", "Camera 1", "Camera 2"],
                                      width=14, state="readonly")
        self.cam_combo.set("Simulation Rig")
        self.cam_combo.pack(side=tk.LEFT, padx=4)

        self.btn_start_cam = ttk.Button(cam_bar, text="Start Camera", command=self._start_camera_selected)
        self.btn_start_cam.pack(side=tk.LEFT, padx=4)

        self.btn_stop_cam = ttk.Button(cam_bar, text="Stop Camera", command=self._stop_camera)
        self.btn_stop_cam.pack(side=tk.LEFT, padx=4)

        ttk.Label(cam_bar, text="Resolution:", style="Card.TLabel").pack(side=tk.LEFT, padx=(12, 4))
        self.combo_res = ttk.Combobox(cam_bar, width=22, state="readonly")
        self.combo_res.pack(side=tk.LEFT, padx=4)
        self.combo_res.bind("<<ComboboxSelected>>", self._on_resolution_selected)

        self.lbl_feed_status = ttk.Label(cam_bar, text="Active", font=("Segoe UI", 9, "bold"),
                                         foreground=self.accent_green)
        self.lbl_feed_status.pack(side=tk.RIGHT, padx=6)

    def _set_initial_sash_position(self) -> None:
        try:
            self.root.update_idletasks()
            total_w = self.root.winfo_width()
            if total_w > 700 and hasattr(self, "content_paned"):
                sash_x = int(total_w * 0.68)
                self.content_paned.sashpos(0, sash_x)
        except Exception:
            pass

    def _build_control_panel(self, parent: ttk.Frame) -> None:
        table_frame = ttk.LabelFrame(parent, text=" MEASUREMENT RESULTS ", padding=6)
        table_frame.pack(fill=tk.BOTH, expand=True, pady=(0, 6))

        columns = ("SNo", "Motion", "Angle", "Steps")
        self.tree = ttk.Treeview(table_frame, columns=columns, show="headings", height=9,
                                 selectmode="browse")

        self.tree.heading("SNo", text="S.No")
        self.tree.heading("Motion", text="Motion")
        self.tree.heading("Angle", text="Angle")
        self.tree.heading("Steps", text="Steps (Double-Click to Edit)")

        self.tree.column("SNo", width=50, anchor=tk.CENTER)
        self.tree.column("Motion", width=110, anchor=tk.CENTER)
        self.tree.column("Angle", width=95, anchor=tk.CENTER)
        self.tree.column("Steps", width=155, anchor=tk.CENTER)

        tree_scroll = ttk.Scrollbar(table_frame, orient=tk.VERTICAL, command=self.tree.yview)
        self.tree.configure(yscrollcommand=tree_scroll.set)

        self.tree.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
        tree_scroll.pack(side=tk.RIGHT, fill=tk.Y)

        self.tree.bind("<Double-1>", self._on_tree_double_click)

        table_actions = ttk.Frame(table_frame)
        table_actions.pack(side=tk.BOTTOM, fill=tk.X, pady=(4, 0))

        btn_export = ttk.Button(table_actions, text="Export CSV", command=self.export_csv)
        btn_export.pack(side=tk.LEFT, padx=2)

        btn_edit_steps = ttk.Button(table_actions, text="Edit Selected Steps", command=self._edit_selected_steps)
        btn_edit_steps.pack(side=tk.LEFT, padx=4)

        action_frame = ttk.LabelFrame(parent, text=" ANGLE MEASUREMENT CONTROLS ", padding=8)
        action_frame.pack(fill=tk.X, pady=(0, 6))

        self.btn_record_pos = ttk.Button(
            action_frame,
            text="RECORD POSITION AFTER ROTATION (MEASURE ANGLE)",
            style="Success.TButton",
            command=self.manual_record_current_position
        )
        self.btn_record_pos.pack(fill=tk.X, pady=(2, 6))

        steps_box = ttk.Frame(action_frame)
        steps_box.pack(fill=tk.X, pady=3)

        ttk.Label(steps_box, text="Commanded Steps (Table):", style="Card.TLabel").pack(side=tk.LEFT, padx=(0, 6))
        self.spin_steps = ttk.Spinbox(steps_box, from_=1, to=10000, width=8)
        self.spin_steps.set(100)
        self.spin_steps.pack(side=tk.LEFT, padx=4)

        ttk.Button(steps_box, text="50", width=3, command=lambda: self.spin_steps.set(50)).pack(side=tk.LEFT, padx=1)
        ttk.Button(steps_box, text="100", width=3, command=lambda: self.spin_steps.set(100)).pack(side=tk.LEFT, padx=1)
        ttk.Button(steps_box, text="200", width=3, command=lambda: self.spin_steps.set(200)).pack(side=tk.LEFT, padx=1)
        ttk.Button(steps_box, text="512", width=4, command=lambda: self.spin_steps.set(512)).pack(side=tk.LEFT, padx=1)
        ttk.Button(steps_box, text="1024", width=4, command=lambda: self.spin_steps.set(1024)).pack(side=tk.LEFT, padx=1)

        reset_box = ttk.Frame(action_frame)
        reset_box.pack(fill=tk.X, pady=(6, 2))

        self.btn_recalibrate = ttk.Button(
            reset_box,
            text="Set New Origin / Recalibrate",
            command=self.activate_set_origin
        )
        self.btn_recalibrate.pack(side=tk.LEFT, fill=tk.X, expand=True, padx=(0, 2))

        self.btn_reset_points = ttk.Button(
            reset_box,
            text="Clear Points (Keep Origin)",
            command=self.clear_points_keep_origin
        )
        self.btn_reset_points.pack(side=tk.LEFT, fill=tk.X, expand=True, padx=2)

        self.btn_refresh = ttk.Button(
            reset_box,
            text="Refresh / Reset Experiment",
            style="Danger.TButton",
            command=self.reset_experiment
        )
        self.btn_refresh.pack(side=tk.LEFT, fill=tk.X, expand=True, padx=(2, 0))

        notebook = ttk.Notebook(parent)
        notebook.pack(fill=tk.BOTH, expand=False, pady=(0, 2))

        guide_tab = ttk.Frame(notebook, padding=8)
        notebook.add(guide_tab, text=" Workflow Guide ")

        guide_text = (
            "1. Click Origin O on the motor shaft center with the crosshair.\n"
            "2. Click the black dot on the pointer arm to set Point 1 (Baseline).\n"
            "   -> Crosshair disappears automatically!\n"
            "3. Rotate the stepper using your external controller.\n"
            "   -> Software automatically tracks the black dot as it rotates!\n"
            "   -> When motor settles, angle is measured and logged to table!\n"
            "   -> Line 1 disappears when Line 3 is drawn (strictly 2 lines on screen)!\n"
            "   -> (Optional) Click 'RECORD POSITION' to manually force-lock."
        )
        ttk.Label(guide_tab, text=guide_text, justify=tk.LEFT, font=("Segoe UI", 8)).pack(anchor=tk.W)

        sim_tab = ttk.Frame(notebook, padding=8)
        notebook.add(sim_tab, text=" Simulation Rig ")

        sim_box = ttk.Frame(sim_tab)
        sim_box.pack(fill=tk.X, pady=2)

        ttk.Label(sim_box, text="Simulate Move:").pack(side=tk.LEFT, padx=(0, 4))
        self.var_sim_dir = tk.StringVar(value="CW")
        ttk.Radiobutton(sim_box, text="CW", variable=self.var_sim_dir, value="CW").pack(side=tk.LEFT, padx=2)
        ttk.Radiobutton(sim_box, text="CCW", variable=self.var_sim_dir, value="CCW").pack(side=tk.LEFT, padx=2)

        self.btn_sim_move = ttk.Button(sim_box, text="Test Rotate Motor", command=self._simulate_external_motion)
        self.btn_sim_move.pack(side=tk.RIGHT, padx=2)

        serial_tab = ttk.Frame(notebook, padding=8)
        notebook.add(serial_tab, text=" Optional Serial Trigger ")

        ser_row = ttk.Frame(serial_tab)
        ser_row.pack(fill=tk.X, pady=2)

        ttk.Label(ser_row, text="Port:").pack(side=tk.LEFT, padx=(0, 4))
        self.combo_ports = ttk.Combobox(ser_row, width=14)
        self._refresh_serial_ports()
        self.combo_ports.pack(side=tk.LEFT, fill=tk.X, expand=True, padx=2)

        ttk.Button(ser_row, text="Connect", command=self.connect_serial).pack(side=tk.RIGHT, padx=2)

    def _build_status_bar(self, parent: ttk.Frame) -> None:
        status_bar = ttk.Frame(parent, style="Panel.TFrame", padding=(8, 4, 8, 4))
        status_bar.pack(fill=tk.X, pady=(4, 0))

        self.stat_cam = ttk.Label(status_bar, text="Camera: CONNECTED", font=("Segoe UI", 9, "bold"),
                                  foreground=self.accent_green)
        self.stat_cam.pack(side=tk.LEFT, padx=8)

        self.stat_state = ttk.Label(status_bar, text="Mode: SET ORIGIN", font=("Segoe UI", 9, "bold"),
                                    foreground=self.accent_amber)
        self.stat_state.pack(side=tk.LEFT, padx=8)

        self.stat_origin = ttk.Label(status_bar, text="Origin O: Not Set", font=("Segoe UI", 9))
        self.stat_origin.pack(side=tk.LEFT, padx=8)

        self.stat_points = ttk.Label(status_bar, text="Points Marked: 0", font=("Segoe UI", 9))
        self.stat_points.pack(side=tk.LEFT, padx=8)

        self.stat_last_measured = ttk.Label(status_bar, text="Last Measured: --°", font=("Segoe UI", 9, "bold"),
                                            foreground=self.accent_amber)
        self.stat_last_measured.pack(side=tk.RIGHT, padx=8)

    def activate_set_origin(self) -> None:
        self.state = AppState.SET_ORIGIN
        self.show_crosshair = True
        self.video_canvas.config(cursor="crosshair")
        self.stat_state.config(text="Mode: SET ORIGIN", foreground=self.accent_amber)
        self.lbl_instruction.config(
            text="STEP 1 OF 2: Aim crosshair at motor shaft center (Origin O) and click.",
            foreground=self.accent_amber
        )

    def activate_set_target(self) -> None:
        self.state = AppState.SET_TARGET
        self.show_crosshair = True
        self.video_canvas.config(cursor="crosshair")
        self.stat_state.config(text="Mode: SET TARGET (Point 1)", foreground=self.accent_amber)
        self.lbl_instruction.config(
            text="STEP 2 OF 2: Aim crosshair at the black dot on the pointer arm (Point 1) and click.",
            foreground=self.accent_amber
        )

    def manual_record_current_position(self) -> None:
        if self.vision_tracker.origin is None:
            messagebox.showwarning("Origin Required", "Please click the motor shaft center (Origin O) first!")
            self.activate_set_origin()
            return

        if len(self.vision_tracker.locked_positions) == 0:
            messagebox.showwarning("Point 1 Required", "Please click the initial black dot position (Point 1) first!")
            self.activate_set_target()
            return

        rec = self.vision_tracker.lock_position()
        if rec is not None:
            self._record_measurement_row(rec)
            self.lbl_instruction.config(
                text=f"Rotation {rec['rotation_num']} recorded: {rec['angle_deg']:.2f}° ({rec['motion']}). Ready for next rotation.",
                foreground=self.accent_green
            )
        else:
            messagebox.showinfo("Position Stable", "Pointer has not moved enough from the previous position to record a new angle.")

    def clear_points_keep_origin(self) -> None:
        self.vision_tracker.reset_positions_keep_origin()
        for item in self.tree.get_children():
            self.tree.delete(item)
        self.measurements.clear()

        self.stat_points.config(text="Points Marked: 0")
        self.stat_last_measured.config(text="Last Measured: --°")

        if self.vision_tracker.origin is not None:
            self.activate_set_target()
        else:
            self.activate_set_origin()

    def reset_experiment(self) -> None:
        self.vision_tracker.clear()
        for item in self.tree.get_children():
            self.tree.delete(item)
        self.measurements.clear()

        self.stat_origin.config(text="Origin O: Not Set")
        self.stat_points.config(text="Points Marked: 0")
        self.stat_last_measured.config(text="Last Measured: --°")

        self.activate_set_origin()

    def _on_canvas_mouse_move(self, event: tk.Event) -> None:
        self.is_mouse_over_feed = True
        fx, fy = self._canvas_to_frame_coords(event.x, event.y)
        self.mouse_pos = (fx, fy)

    def _on_canvas_mouse_leave(self, event: tk.Event) -> None:
        self.is_mouse_over_feed = False
        self.mouse_pos = None

    def _on_canvas_click(self, event: tk.Event) -> None:
        fx, fy = self._canvas_to_frame_coords(event.x, event.y)
        click_pt = (fx, fy)

        if self.state == AppState.SET_ORIGIN:

            self.vision_tracker.set_origin(click_pt)
            self.stat_origin.config(text=f"Origin O: ({fx}, {fy})")

            self.activate_set_target()

        elif self.state == AppState.SET_TARGET:

            ret, frame = self.camera_mgr.read()
            if ret and frame is not None:
                self.vision_tracker.set_initial_target(frame, click_pt)
            else:
                self.vision_tracker.set_initial_target(np.zeros((720, 1280, 3), dtype=np.uint8), click_pt)

            self.stat_points.config(text="Points Marked: 1")

            self.show_crosshair = False
            self.video_canvas.config(cursor="")

            self.state = AppState.ARMED
            self.stat_state.config(text="Mode: ARMED (Auto-Tracking Active)", foreground=self.accent_green)
            self.lbl_instruction.config(
                text="Point 1 locked! Rotate motor with external controller. System will auto-track black dot and measure angles.",
                foreground=self.accent_green
            )

    def _record_measurement_row(self, rec: Dict[str, Any]) -> None:
        sno = rec["rotation_num"]
        motion = rec["motion"]
        angle_display = f"{rec['angle_deg']:.2f}°"

        try:
            steps_display = int(self.spin_steps.get())
        except ValueError:
            steps_display = 100

        record = {
            "sno": sno,
            "motion": motion,
            "angle": angle_display,
            "steps": steps_display
        }
        self.measurements.append(record)

        self.tree.insert("", tk.END, iid=str(sno),
                         values=(sno, motion, angle_display, steps_display))
        self.tree.see(str(sno))

        num_pts = len(self.vision_tracker.locked_positions)
        self.stat_points.config(text=f"Points Marked: {num_pts}")
        self.stat_last_measured.config(text=f"Last Measured: {angle_display}")

    def _canvas_to_frame_coords(self, cx: int, cy: int) -> Tuple[int, int]:
        raw_w, raw_h = self.last_frame_dims
        fx = int((cx - self.disp_offset_x) * self.disp_scale_x)
        fy = int((cy - self.disp_offset_y) * self.disp_scale_y)
        fx = max(0, min(raw_w - 1, fx))
        fy = max(0, min(raw_h - 1, fy))
        return fx, fy

    def _on_tree_double_click(self, event: tk.Event) -> None:
        region = self.tree.identify("region", event.x, event.y)
        if region != "cell":
            return
        col = self.tree.identify_column(event.x)
        if col != "#4":
            return
        row_id = self.tree.identify_row(event.y)
        if not row_id:
            return
        self._spawn_inline_steps_editor(row_id, col)

    def _edit_selected_steps(self) -> None:
        selected = self.tree.selection()
        if not selected:
            messagebox.showinfo("Select Row", "Please select a row in the table first.")
            return
        self._spawn_inline_steps_editor(selected[0], "#4")

    def _spawn_inline_steps_editor(self, row_id: str, col: str) -> None:
        bbox = self.tree.bbox(row_id, col)
        if not bbox:
            return
        x, y, w, h = bbox

        current_vals = list(self.tree.item(row_id, "values"))
        current_steps = current_vals[3]

        entry = ttk.Entry(self.tree, font=("Segoe UI", 9, "bold"))
        entry.insert(0, str(current_steps))
        entry.select_range(0, tk.END)
        entry.focus_set()

        def commit_edit(e=None):
            new_text = entry.get().strip()
            try:
                new_steps_val = int(new_text)
                if new_steps_val <= 0:
                    raise ValueError
                current_vals[3] = str(new_steps_val)

                self.tree.item(row_id, values=current_vals)

                for rec in self.measurements:
                    if str(rec["sno"]) == str(row_id):
                        rec["steps"] = new_steps_val
                        break
            except ValueError:
                pass
            entry.destroy()

        entry.bind("<Return>", commit_edit)
        entry.bind("<FocusOut>", commit_edit)
        entry.bind("<Escape>", lambda e: entry.destroy())
        entry.place(x=x, y=y, width=w, height=h)

    def export_csv(self) -> None:
        if not self.measurements:
            messagebox.showinfo("Export CSV", "No measurement data to export yet.")
            return

        file_path = filedialog.asksaveasfilename(
            title="Export Measurement Results",
            defaultextension=".csv",
            filetypes=[("CSV Files (*.csv)", "*.csv"), ("All Files", "*.*")]
        )
        if not file_path:
            return

        try:
            with open(file_path, mode="w", newline="", encoding="utf-8") as f:
                writer = csv.writer(f)
                writer.writerow(["S.No", "Motion", "Angle", "Steps"])
                for item in self.tree.get_children():
                    vals = self.tree.item(item, "values")
                    writer.writerow(vals)
            messagebox.showinfo("Export Successful", f"Saved {len(self.measurements)} records to:\n{file_path}")
        except Exception as ex:
            messagebox.showerror("Export Failed", f"Failed to save CSV file:\n{str(ex)}")

    def _update_loop(self) -> None:
        while not self.event_queue.empty():
            try:
                ev_type, ev_data = self.event_queue.get_nowait()
                if ev_type == "DONE":
                    self.manual_record_current_position()
                elif ev_type == "ERROR":
                    messagebox.showwarning("Hardware Status", str(ev_data))
            except queue.Empty:
                break

        ret, frame = self.camera_mgr.read()

        if ret and frame is not None:
            raw_h, raw_w = frame.shape[:2]
            self.last_frame_dims = (raw_w, raw_h)

            if self.state in (AppState.ARMED, AppState.MOVING):
                new_meas = self.vision_tracker.process_frame(frame)
                if new_meas is not None:
                    self._record_measurement_row(new_meas)
                    self.lbl_instruction.config(
                        text=f"Rotation {new_meas['rotation_num']} recorded: {new_meas['angle_deg']:.2f}° ({new_meas['motion']}). Ready for next rotation.",
                        foreground=self.accent_green
                    )

                if self.vision_tracker.is_moving:
                    self.state = AppState.MOVING
                    self.stat_state.config(text="Mode: ROTATING (Tracking Line)", foreground=self.accent_amber)
                else:
                    self.state = AppState.ARMED
                    self.stat_state.config(text="Mode: ARMED (Auto-Tracking Active)", foreground=self.accent_green)

            self.vision_tracker.render_overlays(
                frame,
                mouse_pos=self.mouse_pos if self.is_mouse_over_feed else None,
                show_crosshair=self.show_crosshair
            )

            step_instruction = self.lbl_instruction.cget("text")
            VisionTracker.draw_hud(frame, step_instruction, self.camera_mgr.fps)

            self._render_frame_to_canvas(frame)

        self.root.after(config.FRAME_UPDATE_INTERVAL_MS, self._update_loop)

    def _render_frame_to_canvas(self, frame_bgr: np.ndarray) -> None:
        cw = self.video_canvas.winfo_width()
        ch = self.video_canvas.winfo_height()
        if cw < 50 or ch < 50:
            return

        fh, fw = frame_bgr.shape[:2]

        scale = min(cw / fw, ch / fh)
        dw = int(fw * scale)
        dh = int(fh * scale)

        self.disp_scale_x = fw / float(dw)
        self.disp_scale_y = fh / float(dh)
        self.disp_offset_x = (cw - dw) // 2
        self.disp_offset_y = (ch - dh) // 2

        resized = cv2.resize(frame_bgr, (dw, dh), interpolation=cv2.INTER_LINEAR)
        rgb_frame = cv2.cvtColor(resized, cv2.COLOR_BGR2RGB)

        pil_img = Image.fromarray(rgb_frame)
        self.photo_image = ImageTk.PhotoImage(pil_img)

        self.video_canvas.delete("all")
        self.video_canvas.create_image(self.disp_offset_x, self.disp_offset_y,
                                      image=self.photo_image, anchor=tk.NW)

    def _update_resolution_combobox(self) -> None:
        supported = self.camera_mgr.supported_resolutions
        labels = [item[2] for item in supported] if supported else [
            "4K UHD (3840x2160)", "1080p FHD (1920x1080)", "720p HD (1280x720)"
        ]
        self.combo_res["values"] = labels

        curr_w, curr_h = self.camera_mgr.current_width, self.camera_mgr.current_height
        matched = None
        for w, h, lbl in supported:
            if w == curr_w and h == curr_h:
                matched = lbl
                break

        if not matched and labels:
            matched = labels[0]

        if matched:
            self.combo_res.set(matched)
            res_tag = matched.split("(")[0].strip()
            self.lbl_feed_status.config(
                text=f"{res_tag} ({curr_w}x{curr_h}) Active",
                foreground=self.accent_green
            )

    def _on_resolution_selected(self, event=None) -> None:
        selected_text = self.combo_res.get()
        if not selected_text:
            return

        w, h = None, None
        for res_w, res_h, res_lbl in self.camera_mgr.supported_resolutions:
            if res_lbl == selected_text:
                w, h = res_w, res_h
                break

        if w is None or h is None:
            try:
                dims_part = selected_text.split("(")[-1].split(")")[0]
                w_str, h_str = dims_part.split("x")
                w, h = int(w_str), int(h_str)
            except Exception:
                return

        old_w, old_h = self.last_frame_dims
        act_w, act_h = self.camera_mgr.set_resolution(w, h)
        if old_w > 0 and old_h > 0 and (old_w != act_w or old_h != act_h):
            self.vision_tracker.rescale(old_w, old_h, act_w, act_h)
            self.last_frame_dims = (act_w, act_h)

        res_tag = selected_text.split("(")[0].strip()
        self.lbl_feed_status.config(
            text=f"{res_tag} ({act_w}x{act_h}) Active",
            foreground=self.accent_green
        )

    def _start_camera_initial(self) -> None:
        avail = CameraManager.list_available_cameras(2)
        if avail:
            self.cam_combo.set(f"Camera {avail[0]}")
            self.camera_mgr.start(camera_index=avail[0])
            self._update_resolution_combobox()
            self.stat_cam.config(text=f"Camera: CONNECTED (Camera {avail[0]})", foreground=self.accent_green)
        else:
            self.cam_combo.set("Simulation Rig")
            self.camera_mgr.start(force_synthetic=True)
            self._update_resolution_combobox()
            self.stat_cam.config(text="Camera: CONNECTED (Simulation Rig)", foreground=self.accent_green)

        if self.camera_mgr.synthetic_cam:
            self.serial_ctrl.set_synthetic_camera(self.camera_mgr.synthetic_cam)
            self.serial_ctrl.connect("", simulate=True)

    def _start_camera_selected(self) -> None:
        selection = self.cam_combo.get()
        if "Simulation" in selection:
            self.camera_mgr.start(force_synthetic=True)
            self._update_resolution_combobox()
            self.stat_cam.config(text="Camera: CONNECTED (Simulation Rig)", foreground=self.accent_green)
            if self.camera_mgr.synthetic_cam:
                self.serial_ctrl.set_synthetic_camera(self.camera_mgr.synthetic_cam)
        else:
            try:
                idx = int(selection.split()[-1])
                self.camera_mgr.start(camera_index=idx)
                self._update_resolution_combobox()
                self.stat_cam.config(text=f"Camera: CONNECTED (Camera {idx})", foreground=self.accent_green)
            except Exception as ex:
                messagebox.showerror("Camera Error", f"Unable to start camera: {str(ex)}")

    def _stop_camera(self) -> None:
        self.camera_mgr.stop()
        self.stat_cam.config(text="Camera: DISCONNECTED", foreground=self.accent_red)

    def _simulate_external_motion(self) -> None:
        if self.camera_mgr.synthetic_cam is None:
            messagebox.showinfo("Simulation Rig", "Switch camera to 'Simulation Rig' to test simulated rotations.")
            return

        direction = self.var_sim_dir.get()
        try:
            steps = int(self.spin_steps.get())
        except ValueError:
            steps = 100

        self.camera_mgr.synthetic_cam.command_move(direction, steps)

    def _refresh_serial_ports(self) -> None:
        ports = SerialController.list_available_ports()
        self.combo_ports["values"] = ports
        if ports:
            self.combo_ports.set(ports[0])
        else:
            self.combo_ports.set("")

    def connect_serial(self) -> None:
        port_entry = self.combo_ports.get()
        if not port_entry:
            return
        success, msg = self.serial_ctrl.connect(port_entry, baudrate=config.DEFAULT_BAUDRATE, simulate=False)
        if success:
            messagebox.showinfo("Serial", f"Connected to {port_entry}")
        else:
            messagebox.showerror("Serial Connection Failed", msg)

    def _setup_serial_callbacks(self) -> None:
        self.serial_ctrl.on_done_callback = lambda: self.event_queue.put(("DONE", None))
        self.serial_ctrl.on_error_callback = lambda err: self.event_queue.put(("ERROR", err))

    def on_closing(self) -> None:
        try:
            self.camera_mgr.stop()
            self.serial_ctrl.disconnect()
        except Exception:
            pass
        self.root.destroy()
