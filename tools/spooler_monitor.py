#!/usr/bin/env python3
"""
SPOOLER Monitor
Modbus TCP register viewer and editor for the SPOOLER wire tension controller.
No external dependencies — uses only Python standard library.
"""

import tkinter as tk
from tkinter import ttk, messagebox, simpledialog
import socket
import struct
import threading

# ============================================================
# Configuration
# ============================================================
DEFAULT_IP   = '192.168.100.2'
DEFAULT_PORT = 502
POLL_MS      = 500   # Display refresh interval in milliseconds

FAULT_CODES = {0: 'None', 1: 'Wire break', 2: 'Over-travel'}

# ============================================================
# Minimal Modbus TCP Client
# ============================================================

class ModbusTCPClient:
    """Minimal Modbus TCP client using only the standard library."""

    def __init__(self, host, port=502, timeout=2.0):
        self.host    = host
        self.port    = port
        self.timeout = timeout
        self._sock   = None
        self._tid    = 0
        self._lock   = threading.Lock()

    def connect(self):
        try:
            s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            s.settimeout(self.timeout)
            s.connect((self.host, self.port))
            self._sock = s
            return True
        except Exception:
            self._sock = None
            return False

    def disconnect(self):
        if self._sock:
            try:
                self._sock.close()
            except Exception:
                pass
            self._sock = None

    def is_connected(self):
        return self._sock is not None

    # ---- Internal ----

    def _next_tid(self):
        self._tid = (self._tid + 1) & 0xFFFF
        return self._tid

    def _send_recv(self, pdu):
        """Send a Modbus PDU and return the response PDU, or None on error."""
        with self._lock:
            if not self._sock:
                return None
            tid = self._next_tid()
            # MBAP: transaction_id(2) protocol_id(2) length(2) unit_id(1)
            frame = struct.pack('>HHHB', tid, 0, len(pdu) + 1, 1) + pdu
            try:
                self._sock.sendall(frame)
                # Read 7-byte MBAP header
                header = self._recv_exact(7)
                if header is None:
                    raise ConnectionError
                _, _, length, _ = struct.unpack('>HHHB', header)
                # Read PDU (length includes unit_id byte)
                resp_pdu = self._recv_exact(length - 1)
                if resp_pdu is None:
                    raise ConnectionError
                if resp_pdu[0] & 0x80:
                    return None   # Modbus exception response
                return resp_pdu
            except Exception:
                self.disconnect()
                return None

    def _recv_exact(self, n):
        """Read exactly n bytes from the socket."""
        data = b''
        while len(data) < n:
            chunk = self._sock.recv(n - len(data))
            if not chunk:
                return None
            data += chunk
        return data

    # ---- Public read/write ----

    def read_holding_registers(self, start, count):
        resp = self._send_recv(struct.pack('>BHH', 0x03, start, count))
        if resp is None or len(resp) < 2 + count * 2:
            return None
        return [struct.unpack('>H', resp[2 + i*2: 4 + i*2])[0] for i in range(count)]

    def read_input_registers(self, start, count):
        resp = self._send_recv(struct.pack('>BHH', 0x04, start, count))
        if resp is None or len(resp) < 2 + count * 2:
            return None
        return [struct.unpack('>H', resp[2 + i*2: 4 + i*2])[0] for i in range(count)]

    def read_coils(self, start, count):
        resp = self._send_recv(struct.pack('>BHH', 0x01, start, count))
        if resp is None or len(resp) < 2:
            return None
        return [(resp[2 + i // 8] >> (i % 8)) & 1 for i in range(count)]

    def read_discrete_inputs(self, start, count):
        resp = self._send_recv(struct.pack('>BHH', 0x02, start, count))
        if resp is None or len(resp) < 2:
            return None
        return [(resp[2 + i // 8] >> (i % 8)) & 1 for i in range(count)]

    def write_single_coil(self, addr, value):
        resp = self._send_recv(struct.pack('>BHH', 0x05, addr, 0xFF00 if value else 0x0000))
        return resp is not None

    def write_single_register(self, addr, value):
        resp = self._send_recv(struct.pack('>BHH', 0x06, addr, int(value) & 0xFFFF))
        return resp is not None


# ============================================================
# GUI
# ============================================================

class SpoolerMonitor:
    def __init__(self, root):
        self.root   = root
        self.root.title("SPOOLER Monitor")
        self.root.resizable(False, False)
        self.client = None

        self._build_ui()
        self._schedule_poll()

    # ---- UI Construction ----

    def _build_ui(self):
        P = dict(padx=8, pady=4)

        # -- Connection bar --
        cf = ttk.LabelFrame(self.root, text="Connection")
        cf.grid(row=0, column=0, sticky='ew', padx=8, pady=6)

        ttk.Label(cf, text="IP:").grid(row=0, column=0, **P)
        self.ip_var = tk.StringVar(value=DEFAULT_IP)
        ttk.Entry(cf, textvariable=self.ip_var, width=16).grid(row=0, column=1, **P)

        ttk.Label(cf, text="Port:").grid(row=0, column=2, **P)
        self.port_var = tk.StringVar(value=str(DEFAULT_PORT))
        ttk.Entry(cf, textvariable=self.port_var, width=6).grid(row=0, column=3, **P)

        self.conn_btn = ttk.Button(cf, text="Connect", command=self._toggle_connect)
        self.conn_btn.grid(row=0, column=4, **P)

        self.conn_lbl = ttk.Label(cf, text="Disconnected", foreground='red', width=18)
        self.conn_lbl.grid(row=0, column=5, **P)

        # -- Status --
        sf = ttk.LabelFrame(self.root, text="Status")
        sf.grid(row=1, column=0, sticky='ew', padx=8, pady=4)

        ttk.Label(sf, text="Wire Fault:").grid(row=0, column=0, sticky='w', **P)
        self.fault_lbl = ttk.Label(sf, text="—", width=12)
        self.fault_lbl.grid(row=0, column=1, sticky='w', **P)

        ttk.Label(sf, text="Mode:").grid(row=0, column=2, sticky='w', **P)
        self.mode_lbl = ttk.Label(sf, text="—", width=10)
        self.mode_lbl.grid(row=0, column=3, sticky='w', **P)

        ttk.Label(sf, text="Fault Code:").grid(row=1, column=0, sticky='w', **P)
        self.fcode_lbl = ttk.Label(sf, text="—", width=12)
        self.fcode_lbl.grid(row=1, column=1, sticky='w', **P)

        ttk.Button(sf, text="Clear Fault",
                   command=lambda: self._write_reg(7, 1)).grid(row=1, column=2, **P)
        ttk.Button(sf, text="Reset Spools",
                   command=lambda: self._write_reg(8, 1)).grid(row=1, column=3, **P)

        # -- Motor control --
        mf = ttk.LabelFrame(self.root, text="Motor Control")
        mf.grid(row=2, column=0, sticky='ew', padx=8, pady=4)

        ttk.Label(mf, text="Wire Feed:").grid(row=0, column=0, sticky='w', **P)
        self.feed_btn = ttk.Button(mf, text="OFF  ▶  Turn ON",
                                   command=self._toggle_wire_feed)
        self.feed_btn.grid(row=0, column=1, columnspan=2, sticky='w', **P)

        ttk.Label(mf, text="Feed Rate:").grid(row=1, column=0, sticky='w', **P)
        self.feed_rate_lbl = ttk.Label(mf, text="—", width=18)
        self.feed_rate_lbl.grid(row=1, column=1, sticky='w', **P)
        ttk.Button(mf, text="Set…",
                   command=lambda: self._set_value(
                       "Feed Rate", 0, scale=100, decimals=2,
                       min_val=0, max_val=60, unit="rev/min"
                   )).grid(row=1, column=2, **P)

        # -- Dancer --
        df = ttk.LabelFrame(self.root, text="Dancer Position")
        df.grid(row=3, column=0, sticky='ew', padx=8, pady=4)

        dancer_rows = [
            ("Position (live):", None, 'dancer_pos_lbl'),
            ("Setpoint:",         1,   'setpoint_lbl'),
            ("Min Position:",     2,   'min_pos_lbl'),
            ("Max Position:",     3,   'max_pos_lbl'),
        ]
        for i, (label, reg, attr) in enumerate(dancer_rows):
            ttk.Label(df, text=label).grid(row=i, column=0, sticky='w', **P)
            lbl = ttk.Label(df, text="—", width=18)
            lbl.grid(row=i, column=1, sticky='w', **P)
            setattr(self, attr, lbl)
            if reg is not None:
                r = reg
                ttk.Button(df, text="Set…",
                           command=lambda r=r, n=label: self._set_value(
                               n.rstrip(':'), r, scale=100, decimals=1,
                               min_val=0, max_val=75, unit="mm"
                           )).grid(row=i, column=2, **P)

        # -- PID --
        pf = ttk.LabelFrame(self.root, text="PID Gains")
        pf.grid(row=4, column=0, sticky='ew', padx=8, pady=4)

        pid_rows = [
            ("KP:", 4, 'kp_lbl'),
            ("KI:", 5, 'ki_lbl'),
            ("KD:", 6, 'kd_lbl'),
        ]
        for i, (label, reg, attr) in enumerate(pid_rows):
            ttk.Label(pf, text=label).grid(row=i, column=0, sticky='w', **P)
            lbl = ttk.Label(pf, text="—", width=18)
            lbl.grid(row=i, column=1, sticky='w', **P)
            setattr(self, attr, lbl)
            r = reg
            ttk.Button(pf, text="Set…",
                       command=lambda r=r, n=label: self._set_value(
                           f"PID {n.rstrip(':')}", r, scale=1000, decimals=3,
                           min_val=0, max_val=100
                       )).grid(row=i, column=2, **P)

        # -- Water quality --
        wf = ttk.LabelFrame(self.root, text="Water Quality")
        wf.grid(row=5, column=0, sticky='ew', padx=8, pady=4)

        ttk.Label(wf, text="TDS (raw ADC 0–4095):").grid(row=0, column=0, sticky='w', **P)
        self.tds_lbl = ttk.Label(wf, text="—", width=18)
        self.tds_lbl.grid(row=0, column=1, sticky='w', **P)

        # -- Manual inputs debug panel --
        bf = ttk.LabelFrame(self.root, text="Manual Inputs (debug)")
        bf.grid(row=6, column=0, sticky='ew', padx=8, pady=4)

        LED_SIZE = 18

        ttk.Label(bf, text="Feed button:").grid(row=0, column=0, sticky='w', **P)
        self.feed_btn_canvas = tk.Canvas(bf, width=LED_SIZE, height=LED_SIZE,
                                         highlightthickness=0)
        self.feed_btn_canvas.grid(row=0, column=1, sticky='w', padx=4, pady=4)
        self.feed_btn_led = self.feed_btn_canvas.create_oval(
            2, 2, LED_SIZE - 2, LED_SIZE - 2, fill='gray', outline='black')

        ttk.Label(bf, text="Recover button:").grid(row=0, column=2, sticky='w', **P)
        self.recover_btn_canvas = tk.Canvas(bf, width=LED_SIZE, height=LED_SIZE,
                                             highlightthickness=0)
        self.recover_btn_canvas.grid(row=0, column=3, sticky='w', padx=4, pady=4)
        self.recover_btn_led = self.recover_btn_canvas.create_oval(
            2, 2, LED_SIZE - 2, LED_SIZE - 2, fill='gray', outline='black')

        # -- Footer --
        ttk.Label(self.root, text=f"Refreshing every {POLL_MS} ms  •  No external dependencies",
                  foreground='gray').grid(row=7, column=0, pady=6)

    # ---- Connection ----

    def _toggle_connect(self):
        if self.client and self.client.is_connected():
            self.client.disconnect()
            self.client = None
            self.conn_btn.config(text="Connect")
            self.conn_lbl.config(text="Disconnected", foreground='red')
            self._clear_display()
        else:
            try:
                port = int(self.port_var.get())
            except ValueError:
                messagebox.showerror("Error", "Invalid port number")
                return
            self.conn_lbl.config(text="Connecting…", foreground='orange')
            self.root.update()
            self.client = ModbusTCPClient(self.ip_var.get(), port)
            if self.client.connect():
                self.conn_btn.config(text="Disconnect")
                self.conn_lbl.config(text="Connected", foreground='green')
            else:
                self.client = None
                self.conn_lbl.config(text="Failed to connect", foreground='red')

    def _clear_display(self):
        for attr in ('fault_lbl', 'mode_lbl', 'fcode_lbl', 'feed_rate_lbl',
                     'dancer_pos_lbl', 'setpoint_lbl', 'min_pos_lbl', 'max_pos_lbl',
                     'kp_lbl', 'ki_lbl', 'kd_lbl', 'tds_lbl'):
            getattr(self, attr).config(text="—", foreground='black')
        self.feed_btn.config(text="OFF  ▶  Turn ON")
        self.feed_btn_canvas.itemconfig(self.feed_btn_led, fill='gray')
        self.recover_btn_canvas.itemconfig(self.recover_btn_led, fill='gray')

    # ---- Polling ----

    def _schedule_poll(self):
        self.root.after(POLL_MS, self._poll)

    def _poll(self):
        if self.client and self.client.is_connected():
            try:
                self._refresh()
            except Exception:
                self.conn_lbl.config(text="Connection lost", foreground='red')
                self.client = None
                self.conn_btn.config(text="Connect")
                self._clear_display()
        self._schedule_poll()

    def _refresh(self):
        hr    = self.client.read_holding_registers(0, 9)
        ir    = self.client.read_input_registers(0, 5)
        coils = self.client.read_coils(0, 1)
        di    = self.client.read_discrete_inputs(0, 2)

        if hr:
            self.feed_rate_lbl.config(text=f"{hr[0] / 100:.2f} rev/min")
            self.setpoint_lbl.config( text=f"{hr[1] / 100:.1f} mm")
            self.min_pos_lbl.config(  text=f"{hr[2] / 100:.1f} mm")
            self.max_pos_lbl.config(  text=f"{hr[3] / 100:.1f} mm")
            self.kp_lbl.config(       text=f"{hr[4] / 1000:.3f}")
            self.ki_lbl.config(       text=f"{hr[5] / 1000:.3f}")
            self.kd_lbl.config(       text=f"{hr[6] / 1000:.3f}")

        if ir:
            self.dancer_pos_lbl.config(text=f"{ir[0] / 100:.1f} mm")
            self.tds_lbl.config(       text=str(ir[1]))
            code = ir[2]
            self.fcode_lbl.config(
                text=FAULT_CODES.get(code, f"Unknown ({code})"),
                foreground='red' if code else 'black'
            )
            feed_pressed    = bool(ir[3])
            recover_pressed = bool(ir[4])
            self.feed_btn_canvas.itemconfig(
                self.feed_btn_led, fill='lime green' if feed_pressed else 'gray')
            self.recover_btn_canvas.itemconfig(
                self.recover_btn_led, fill='lime green' if recover_pressed else 'gray')

        if coils:
            on = bool(coils[0])
            self.feed_btn.config(
                text="ON  ◼  Turn OFF" if on else "OFF  ▶  Turn ON",
                style='TButton'
            )

        if di:
            fault = bool(di[0])
            self.fault_lbl.config(
                text="YES" if fault else "NO",
                foreground='red' if fault else 'green'
            )
            auto = bool(di[1])
            self.mode_lbl.config(
                text="Auto" if auto else "Manual",
                foreground='black'
            )

    # ---- Write helpers ----

    def _check_connected(self):
        if not self.client or not self.client.is_connected():
            messagebox.showwarning("Not connected", "Connect to the SPOOLER first.")
            return False
        return True

    def _write_reg(self, addr, value):
        if not self._check_connected():
            return
        self.client.write_single_register(addr, value)

    def _toggle_wire_feed(self):
        if not self._check_connected():
            return
        coils = self.client.read_coils(0, 1)
        if coils is not None:
            self.client.write_single_coil(0, not bool(coils[0]))

    def _set_value(self, name, reg_addr, scale, decimals, min_val, max_val, unit=''):
        """Prompt the user for a new value and write it to a holding register."""
        if not self._check_connected():
            return

        hr = self.client.read_holding_registers(reg_addr, 1)
        current_str = f"{hr[0] / scale:.{decimals}f}" if hr else "?"
        unit_str = f" {unit}" if unit else ""

        prompt = (f"Enter new value for {name}\n"
                  f"Range: {min_val} – {max_val}{unit_str}\n"
                  f"Current: {current_str}{unit_str}")

        result = simpledialog.askstring("Set Value", prompt, parent=self.root)
        if result is None:
            return
        try:
            val = float(result)
        except ValueError:
            messagebox.showerror("Invalid input", "Please enter a number.")
            return
        if not (min_val <= val <= max_val):
            messagebox.showerror("Out of range",
                                 f"Value must be between {min_val} and {max_val}{unit_str}.")
            return

        raw = int(round(val * scale))
        self.client.write_single_register(reg_addr, raw)


# ============================================================
# Entry point
# ============================================================

if __name__ == '__main__':
    root = tk.Tk()
    app  = SpoolerMonitor(root)
    root.mainloop()
