import queue
import threading
import time
from typing import List, Optional, Callable
import serial
import serial.tools.list_ports

import config

class SerialController:

    def __init__(self):
        self.ser: Optional[serial.Serial] = None
        self.is_connected: bool = False
        self.is_simulated: bool = False
        self.port: str = ""
        self.baudrate: int = config.DEFAULT_BAUDRATE

        self._rx_thread: Optional[threading.Thread] = None
        self._running: bool = False

        self.incoming_queue: queue.Queue = queue.Queue()
        self.on_done_callback: Optional[Callable[[], None]] = None
        self.on_status_callback: Optional[Callable[[str], None]] = None
        self.on_error_callback: Optional[Callable[[str], None]] = None

        self._synthetic_camera = None

    def set_synthetic_camera(self, syn_cam) -> None:
        self._synthetic_camera = syn_cam

    @staticmethod
    def list_available_ports() -> List[str]:
        ports = serial.tools.list_ports.comports()
        port_list = []
        for p in ports:
            port_list.append(f"{p.device} - {p.description}")
        return port_list

    def connect(self, port: str, baudrate: int = config.DEFAULT_BAUDRATE,
                simulate: bool = False) -> Tuple[bool, str]:

        self.disconnect()
        self.port = port
        self.baudrate = baudrate
        self.is_simulated = simulate

        if self.is_simulated:
            self.is_connected = True
            self._running = True
            if self.on_status_callback:
                self.on_status_callback("SIMULATION CONNECTED")
            return True, "Simulation mode active"

        try:

            clean_port = port.split()[0] if port else ""
            if not clean_port:
                return False, "No COM port specified"

            self.ser = serial.Serial(
                port=clean_port,
                baudrate=baudrate,
                timeout=config.SERIAL_TIMEOUT_SEC,
                write_timeout=1.0
            )

            time.sleep(1.5)

            self.ser.reset_input_buffer()
            self.ser.reset_output_buffer()

            self.is_connected = True
            self._running = True

            self._rx_thread = threading.Thread(target=self._read_loop, daemon=True)
            self._rx_thread.start()

            if self.on_status_callback:
                self.on_status_callback(f"CONNECTED ({clean_port} @ {baudrate})")
            return True, f"Connected to {clean_port}"

        except Exception as ex:
            self.is_connected = False
            self.ser = None
            err_msg = f"Serial connection failed: {str(ex)}"
            if self.on_error_callback:
                self.on_error_callback(err_msg)
            return False, err_msg

    def _read_loop(self) -> None:
        while self._running and self.ser and self.ser.is_open:
            try:
                line = self.ser.readline().decode('utf-8', errors='ignore').strip()
                if line:
                    self.incoming_queue.put(line)
                    self._handle_response_line(line)
            except (serial.SerialException, OSError) as e:
                if self._running:
                    self.is_connected = False
                    if self.on_error_callback:
                        self.on_error_callback(f"Serial disconnected: {str(e)}")
                break
            except Exception:
                pass

    def _handle_response_line(self, line: str) -> None:
        upper = line.upper()
        if "DONE" in upper:
            if self.on_done_callback:
                self.on_done_callback()
        elif "ERROR" in upper:
            if self.on_error_callback:
                self.on_error_callback(f"Hardware Error: {line}")
        elif "READY" in upper or "BUSY" in upper:
            if self.on_status_callback:
                self.on_status_callback(f"Motor: {line}")

    def send_move(self, direction: str, steps: int) -> Tuple[bool, str]:
        dir_clean = "CW" if direction.upper() in ("CW", "CLOCKWISE") else "CCW"
        cmd = f"{dir_clean},{steps}\n"

        if self.is_simulated:

            if self._synthetic_camera:
                self._synthetic_camera.command_move(dir_clean, steps)

            duration = max(0.4, (steps * 1.8) / config.SIMULATION_MOTOR_SPEED_DPS)
            threading.Thread(target=self._simulate_done_after_delay,
                             args=(duration,), daemon=True).start()
            return True, f"Simulated command: {cmd.strip()}"

        if not self.is_connected or not self.ser or not self.ser.is_open:
            return False, "Serial not connected"

        try:
            self.ser.write(cmd.encode('utf-8'))
            self.ser.flush()
            return True, f"Sent: {cmd.strip()}"
        except Exception as ex:
            err_msg = f"Failed to send serial command: {str(ex)}"
            if self.on_error_callback:
                self.on_error_callback(err_msg)
            return False, err_msg

    def _simulate_done_after_delay(self, delay_sec: float) -> None:
        time.sleep(delay_sec)

        time.sleep(0.15)
        if self.on_done_callback:
            self.on_done_callback()

    def send_stop(self) -> None:
        if self.is_simulated:
            if self._synthetic_camera:
                self._synthetic_camera.stop()
            if self.on_done_callback:
                self.on_done_callback()
            return

        if self.ser and self.ser.is_open:
            try:
                self.ser.write(b"STOP\n")
                self.ser.flush()
            except Exception:
                pass

    def disconnect(self) -> None:
        self._running = False
        self.is_connected = False

        if self.ser is not None:
            try:
                self.ser.close()
            except Exception:
                pass
            self.ser = None

        if self._rx_thread and self._rx_thread.is_alive():
            self._rx_thread.join(timeout=0.5)
        self._rx_thread = None

        if self.on_status_callback:
            self.on_status_callback("DISCONNECTED")
