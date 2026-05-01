"""EMM42 V5 Motor Controller GUI — tkinter, no extra dependencies."""
from __future__ import annotations

import threading
import tkinter as tk
from tkinter import ttk, messagebox
import time
from typing import Optional

from emm42 import Session, MotorDevice
from emm42.protocol.codec import ChecksumMode
from emm42.transport import Transport


CHECKSUM_OPTIONS = {
    "FIXED_6B (默认)": ChecksumMode.FIXED_6B,
    "XOR": ChecksumMode.XOR,
    "CRC8": ChecksumMode.CRC8,
    "MODBUS": ChecksumMode.MODBUS,
}

BAUDRATE_OPTIONS = [9600, 19200, 38400, 57600, 115200, 230400]


class App(tk.Tk):
    def __init__(self) -> None:
        super().__init__()
        self.title("EMM42 V5 Motor Controller")
        self.resizable(True, True)
        self.minsize(640, 600)

        self._motor: Optional[MotorDevice] = None
        self._poll_active = False
        self._poll_thread: Optional[threading.Thread] = None

        self._build_ui()
        self._refresh_ports()

    # ── UI construction ────────────────────────────────────────────────────────

    def _build_ui(self) -> None:
        # ── top: connection bar ──────────────────────────────────────────────
        conn_frame = ttk.LabelFrame(self, text="连接设置 Connection", padding=6)
        conn_frame.pack(fill="x", padx=8, pady=(8, 0))

        ttk.Label(conn_frame, text="串口 Port:").grid(row=0, column=0, sticky="e", padx=4)
        self._port_var = tk.StringVar()
        self._port_cb = ttk.Combobox(conn_frame, textvariable=self._port_var, width=12)
        self._port_cb.grid(row=0, column=1, padx=4)
        ttk.Button(conn_frame, text="刷新 Refresh", command=self._refresh_ports).grid(row=0, column=2, padx=4)

        ttk.Label(conn_frame, text="波特率 Baud:").grid(row=0, column=3, sticky="e", padx=4)
        self._baud_var = tk.IntVar(value=115200)
        ttk.Combobox(conn_frame, textvariable=self._baud_var, values=BAUDRATE_OPTIONS, width=9).grid(row=0, column=4, padx=4)

        ttk.Label(conn_frame, text="地址 Addr:").grid(row=0, column=5, sticky="e", padx=4)
        self._addr_var = tk.IntVar(value=1)
        ttk.Spinbox(conn_frame, from_=1, to=255, textvariable=self._addr_var, width=5).grid(row=0, column=6, padx=4)

        ttk.Label(conn_frame, text="校验 Checksum:").grid(row=1, column=0, sticky="e", padx=4, pady=(4, 0))
        self._checksum_var = tk.StringVar(value="FIXED_6B (默认)")
        ttk.Combobox(conn_frame, textvariable=self._checksum_var, values=list(CHECKSUM_OPTIONS), width=16, state="readonly").grid(row=1, column=1, columnspan=2, sticky="w", padx=4, pady=(4, 0))

        ttk.Label(conn_frame, text="超时 Timeout(s):").grid(row=1, column=3, sticky="e", padx=4, pady=(4, 0))
        self._timeout_var = tk.DoubleVar(value=0.5)
        ttk.Entry(conn_frame, textvariable=self._timeout_var, width=6).grid(row=1, column=4, padx=4, pady=(4, 0))

        self._conn_btn = ttk.Button(conn_frame, text="连接 Connect", command=self._toggle_connect)
        self._conn_btn.grid(row=1, column=5, columnspan=2, padx=4, pady=(4, 0))

        self._conn_status = ttk.Label(conn_frame, text="● 未连接", foreground="red")
        self._conn_status.grid(row=0, column=7, rowspan=2, padx=10)

        # ── main content: notebook ───────────────────────────────────────────
        nb = ttk.Notebook(self)
        nb.pack(fill="both", expand=True, padx=8, pady=6)

        self._build_motion_tab(nb)
        self._build_status_tab(nb)
        self._build_homing_tab(nb)

        # ── bottom: log ──────────────────────────────────────────────────────
        log_frame = ttk.LabelFrame(self, text="日志 Log", padding=4)
        log_frame.pack(fill="x", padx=8, pady=(0, 8))
        self._log = tk.Text(log_frame, height=6, state="disabled", font=("Consolas", 9))
        sb = ttk.Scrollbar(log_frame, command=self._log.yview)
        self._log.configure(yscrollcommand=sb.set)
        self._log.pack(side="left", fill="both", expand=True)
        sb.pack(side="right", fill="y")
        ttk.Button(log_frame, text="清空 Clear", command=self._clear_log).pack(side="right", padx=4)

    def _build_motion_tab(self, nb: ttk.Notebook) -> None:
        frame = ttk.Frame(nb, padding=8)
        nb.add(frame, text="运动控制 Motion")

        # Enable / Disable / Stop row
        ctrl_row = ttk.Frame(frame)
        ctrl_row.pack(fill="x", pady=(0, 8))
        ttk.Button(ctrl_row, text="使能 Enable", width=14, command=self._enable).pack(side="left", padx=4)
        ttk.Button(ctrl_row, text="关闭使能 Disable", width=14, command=self._disable).pack(side="left", padx=4)
        ttk.Button(ctrl_row, text="🛑 急停 E-STOP", width=14, command=self._stop,
                   style="Stop.TButton").pack(side="left", padx=4)
        ttk.Button(ctrl_row, text="解除堵转 Clear Stall", width=16, command=self._clear_stall).pack(side="left", padx=4)
        ttk.Button(ctrl_row, text="清零位置 Reset Pos", width=16, command=self._reset_position).pack(side="left", padx=4)

        # Create stop button style
        style = ttk.Style()
        style.configure("Stop.TButton", foreground="red")

        # Velocity mode
        vel_frame = ttk.LabelFrame(frame, text="速度模式 Velocity Mode", padding=8)
        vel_frame.pack(fill="x", pady=4)

        ttk.Label(vel_frame, text="方向 Dir:").grid(row=0, column=0, sticky="e")
        self._vel_dir = tk.IntVar(value=0)
        ttk.Radiobutton(vel_frame, text="顺时针 CW (0)", variable=self._vel_dir, value=0).grid(row=0, column=1, sticky="w")
        ttk.Radiobutton(vel_frame, text="逆时针 CCW (1)", variable=self._vel_dir, value=1).grid(row=0, column=2, sticky="w")

        ttk.Label(vel_frame, text="转速 RPM:").grid(row=1, column=0, sticky="e", pady=4)
        self._vel_rpm = tk.IntVar(value=300)
        ttk.Spinbox(vel_frame, from_=0, to=5000, textvariable=self._vel_rpm, width=8).grid(row=1, column=1, sticky="w")

        ttk.Label(vel_frame, text="加速度 Accel (0–255):").grid(row=1, column=2, sticky="e", padx=(16, 4))
        self._vel_accel = tk.IntVar(value=0)
        ttk.Spinbox(vel_frame, from_=0, to=255, textvariable=self._vel_accel, width=6).grid(row=1, column=3, sticky="w")

        ttk.Button(vel_frame, text="▶ 启动速度模式 Start Velocity", command=self._velocity).grid(row=1, column=4, padx=12)

        # Position mode
        pos_frame = ttk.LabelFrame(frame, text="位置模式 Position Mode", padding=8)
        pos_frame.pack(fill="x", pady=4)

        ttk.Label(pos_frame, text="方向 Dir:").grid(row=0, column=0, sticky="e")
        self._pos_dir = tk.IntVar(value=0)
        ttk.Radiobutton(pos_frame, text="CW (0)", variable=self._pos_dir, value=0).grid(row=0, column=1, sticky="w")
        ttk.Radiobutton(pos_frame, text="CCW (1)", variable=self._pos_dir, value=1).grid(row=0, column=2, sticky="w")

        ttk.Label(pos_frame, text="转速 RPM:").grid(row=1, column=0, sticky="e", pady=4)
        self._pos_rpm = tk.IntVar(value=300)
        ttk.Spinbox(pos_frame, from_=0, to=5000, textvariable=self._pos_rpm, width=8).grid(row=1, column=1, sticky="w")

        ttk.Label(pos_frame, text="加速度 Accel:").grid(row=1, column=2, sticky="e", padx=(16, 4))
        self._pos_accel = tk.IntVar(value=0)
        ttk.Spinbox(pos_frame, from_=0, to=255, textvariable=self._pos_accel, width=6).grid(row=1, column=3, sticky="w")

        ttk.Label(pos_frame, text="脉冲数 Pulses:").grid(row=2, column=0, sticky="e", pady=4)
        self._pos_pulses = tk.IntVar(value=1600)
        ttk.Spinbox(pos_frame, from_=0, to=9999999, textvariable=self._pos_pulses, width=10).grid(row=2, column=1, sticky="w")
        ttk.Label(pos_frame, text="(1600 = 1圈@16细分)").grid(row=2, column=2, sticky="w", padx=4)

        ttk.Label(pos_frame, text="模式 Mode:").grid(row=2, column=3, sticky="e", padx=(16, 4))
        self._pos_relative = tk.BooleanVar(value=True)
        ttk.Checkbutton(pos_frame, text="相对 Relative", variable=self._pos_relative).grid(row=2, column=4, sticky="w")

        ttk.Button(pos_frame, text="▶ 执行位置移动 Move", command=self._position).grid(row=3, column=0, columnspan=5, pady=(4, 0))

    def _build_status_tab(self, nb: ttk.Notebook) -> None:
        frame = ttk.Frame(nb, padding=8)
        nb.add(frame, text="状态监控 Status")

        self._poll_var = tk.BooleanVar(value=False)
        poll_row = ttk.Frame(frame)
        poll_row.pack(fill="x", pady=(0, 8))
        ttk.Checkbutton(poll_row, text="自动轮询 Auto Poll", variable=self._poll_var, command=self._toggle_poll).pack(side="left")
        ttk.Label(poll_row, text="间隔 Interval(s):").pack(side="left", padx=(16, 4))
        self._poll_interval = tk.DoubleVar(value=1.0)
        ttk.Spinbox(poll_row, from_=0.2, to=10.0, increment=0.1, textvariable=self._poll_interval, width=6).pack(side="left")
        ttk.Button(poll_row, text="手动读取 Read Once", command=self._poll_once).pack(side="left", padx=12)

        # Status grid
        fields = [
            ("实时位置 Position (°)", "_sv_position"),
            ("目标位置 Target Pos (°)", "_sv_target_pos"),
            ("位置误差 Pos Error (°)", "_sv_pos_error"),
            ("实时转速 Velocity (RPM)", "_sv_velocity"),
            ("编码器 Encoder", "_sv_encoder"),
            ("总线电压 Bus Voltage (mV)", "_sv_voltage"),
            ("相电流 Phase Current (mA)", "_sv_current"),
            ("使能状态 Enabled", "_sv_enabled"),
            ("到位标志 In Position", "_sv_in_pos"),
            ("堵转 Stalled", "_sv_stalled"),
            ("堵转保护 Stall Prot.", "_sv_stall_prot"),
            ("正在回零 Is Homing", "_sv_is_homing"),
            ("回零失败 Homing Failed", "_sv_hom_failed"),
        ]

        status_frame = ttk.LabelFrame(frame, text="电机状态 Motor State", padding=8)
        status_frame.pack(fill="both", expand=True)
        for i, (label, attr) in enumerate(fields):
            row, col = divmod(i, 2)
            ttk.Label(status_frame, text=label + ":").grid(row=row, column=col * 2, sticky="e", padx=(8, 4), pady=2)
            sv = tk.StringVar(value="—")
            setattr(self, attr, sv)
            ttk.Label(status_frame, textvariable=sv, width=18, anchor="w",
                      relief="sunken", background="white").grid(row=row, column=col * 2 + 1, sticky="w", padx=(0, 16), pady=2)

    def _build_homing_tab(self, nb: ttk.Notebook) -> None:
        frame = ttk.Frame(nb, padding=8)
        nb.add(frame, text="回零 Homing")

        ttk.LabelFrame(frame, text="").pack()  # spacer
        actions = ttk.LabelFrame(frame, text="回零操作 Homing Actions", padding=12)
        actions.pack(fill="x", pady=4)

        ttk.Button(actions, text="设置当前位置为零点\nSet Origin Here", width=22,
                   command=lambda: self._run(lambda: self._motor.homing_set_origin(store=False))).pack(side="left", padx=8)
        ttk.Button(actions, text="触发回零\nTrigger Homing", width=18,
                   command=lambda: self._run(lambda: self._motor.homing_trigger(mode=0))).pack(side="left", padx=8)
        ttk.Button(actions, text="中断回零\nInterrupt Homing", width=18,
                   command=lambda: self._run(lambda: self._motor.homing_interrupt())).pack(side="left", padx=8)

    # ── connection logic ───────────────────────────────────────────────────────

    def _refresh_ports(self) -> None:
        ports = Transport.list_available_ports()
        self._port_cb["values"] = ports
        if ports and not self._port_var.get():
            self._port_var.set(ports[0])
        if not ports:
            self._log_msg("⚠ 未检测到串口设备。请连接硬件后点击刷新。")

    def _toggle_connect(self) -> None:
        if self._motor is None:
            self._connect()
        else:
            self._disconnect()

    def _connect(self) -> None:
        port = self._port_var.get().strip()
        if not port:
            messagebox.showerror("错误", "请选择或输入串口号。")
            return
        try:
            sess = Session(
                port=port,
                baudrate=int(self._baud_var.get()),
                address=int(self._addr_var.get()),
                checksum_mode=CHECKSUM_OPTIONS[self._checksum_var.get()],
                timeout=float(self._timeout_var.get()),
            )
            motor = MotorDevice(sess)
            motor.open()
            self._motor = motor
            self._conn_status.config(text=f"● 已连接 {port}", foreground="green")
            self._conn_btn.config(text="断开 Disconnect")
            self._log_msg(f"✔ 已连接到 {port} @ {self._baud_var.get()} baud, addr={self._addr_var.get()}")
        except Exception as exc:
            self._log_msg(f"✘ 连接失败: {exc}")

    def _disconnect(self) -> None:
        self._stop_poll()
        if self._motor:
            try:
                self._motor.close()
            except Exception:
                pass
            self._motor = None
        self._conn_status.config(text="● 未连接", foreground="red")
        self._conn_btn.config(text="连接 Connect")
        self._log_msg("⏏ 已断开连接。")

    # ── motion commands ────────────────────────────────────────────────────────

    def _run(self, fn) -> None:
        """Execute fn() in a thread; log result or error."""
        if self._motor is None:
            self._log_msg("✘ 未连接，请先连接设备。")
            return
        def _task():
            try:
                result = fn()
                if result is not None:
                    self._log_msg(f"  → {result}")
                else:
                    self._log_msg("  ✔ 命令已发送。")
            except Exception as exc:
                self._log_msg(f"✘ 错误: {exc}")
        threading.Thread(target=_task, daemon=True).start()

    def _enable(self) -> None:
        self._run(lambda: self._motor.enable())

    def _disable(self) -> None:
        self._run(lambda: self._motor.disable())

    def _stop(self) -> None:
        self._run(lambda: self._motor.stop())

    def _clear_stall(self) -> None:
        self._run(lambda: self._motor.clear_stall())

    def _reset_position(self) -> None:
        self._run(lambda: self._motor.reset_position())

    def _velocity(self) -> None:
        self._run(lambda: self._motor.velocity(
            direction=self._vel_dir.get(),
            rpm=self._vel_rpm.get(),
            acceleration=self._vel_accel.get(),
        ))

    def _position(self) -> None:
        self._run(lambda: self._motor.position(
            direction=self._pos_dir.get(),
            rpm=self._pos_rpm.get(),
            acceleration=self._pos_accel.get(),
            pulses=self._pos_pulses.get(),
            relative=self._pos_relative.get(),
        ))

    # ── status polling ─────────────────────────────────────────────────────────

    def _toggle_poll(self) -> None:
        if self._poll_var.get():
            self._start_poll()
        else:
            self._stop_poll()

    def _start_poll(self) -> None:
        self._poll_active = True
        self._poll_thread = threading.Thread(target=self._poll_loop, daemon=True)
        self._poll_thread.start()

    def _stop_poll(self) -> None:
        self._poll_active = False
        self._poll_var.set(False)

    def _poll_loop(self) -> None:
        while self._poll_active:
            if self._motor:
                self._do_poll()
            time.sleep(max(0.2, self._poll_interval.get()))

    def _poll_once(self) -> None:
        threading.Thread(target=self._do_poll, daemon=True).start()

    def _do_poll(self) -> None:
        if not self._motor:
            return
        try:
            pos = self._motor.read_position()
            vel = self._motor.read_velocity()
            status = self._motor.read_status()
            voltage = self._motor.read_bus_voltage()
            current = self._motor.read_phase_current()
            homing = self._motor.read_origin_status()

            self.after(0, lambda: self._update_status_labels(
                pos, vel, status, voltage, current, homing))
        except Exception as exc:
            self.after(0, lambda: self._log_msg(f"✘ 轮询错误: {exc}"))
            self.after(0, self._stop_poll)

    def _update_status_labels(self, pos, vel, status, voltage, current, homing) -> None:
        self._sv_position.set(f"{pos:.2f}")
        self._sv_velocity.set(str(vel))
        self._sv_enabled.set("✔ 是" if status.get("enabled") else "✘ 否")
        self._sv_in_pos.set("✔ 是" if status.get("in_position") else "✘ 否")
        self._sv_stalled.set("⚠ 是" if status.get("stalled") else "否")
        self._sv_stall_prot.set("⚠ 是" if status.get("stall_protection") else "否")
        self._sv_voltage.set(str(voltage))
        self._sv_current.set(str(current))
        self._sv_is_homing.set("✔ 是" if homing.get("is_homing") else "否")
        self._sv_hom_failed.set("✘ 是" if homing.get("homing_failed") else "否")

    # ── log ────────────────────────────────────────────────────────────────────

    def _log_msg(self, msg: str) -> None:
        def _append():
            self._log.config(state="normal")
            ts = time.strftime("%H:%M:%S")
            self._log.insert("end", f"[{ts}] {msg}\n")
            self._log.see("end")
            self._log.config(state="disabled")
        self.after(0, _append)

    def _clear_log(self) -> None:
        self._log.config(state="normal")
        self._log.delete("1.0", "end")
        self._log.config(state="disabled")

    def on_close(self) -> None:
        self._disconnect()
        self.destroy()


if __name__ == "__main__":
    app = App()
    app.protocol("WM_DELETE_WINDOW", app.on_close)
    app.mainloop()
