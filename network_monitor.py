#!/usr/bin/env python3
"""
ESP32-S3 W5500 AUV 网络监视器

实时显示网络连接状态和IP地址信息，
支持通过串口配置TCP对端、发送NMEA-0183控制指令，
支持直接通过TCP网络接收数据并显示来源地址。
"""

import tkinter as tk
from tkinter import ttk, scrolledtext, messagebox, filedialog
import serial
import serial.tools.list_ports
import threading
import socket
import re
import json
from datetime import datetime


class ESP32NetworkMonitor:
    def __init__(self, root):
        self.root = root
        self.root.title("ESP32-S3 AUV 网络监视器")
        self.root.geometry("1200x850")
        self.root.configure(bg='#f0f0f0')

        # 串口
        self.serial_port = None
        self.serial_thread = None
        self.is_running = False

        # 网络状态变量
        self.ip_address = "未获取"
        self.netmask = "未获取"
        self.gateway = "未获取"
        self.mac_address = "未获取"
        self.link_status = "断开"
        self.dhcp_status = "等待中"
        self.uptime = "00:00:00"
        self.link_start_time = None

        # AUV 状态
        self.relay = 0
        self.claw = 0
        self.tcp_connected = False

        # TCP服务端（接收ESP32数据）
        self.tcp_server_sock = None
        self.tcp_server_thread = None
        self.tcp_clients = {}  # sock -> addr
        self.tcp_server_running = False

        # TCP客户端（连接ESP32发送控制）
        self.tcp_client_sock = None
        self.tcp_client_lock = threading.Lock()

        self.setup_ui()

    def setup_ui(self):
        # 创建主框架
        main_frame = ttk.Frame(self.root, padding="10")
        main_frame.grid(row=0, column=0, sticky=(tk.W, tk.E, tk.N, tk.S))
        self.root.columnconfigure(0, weight=1)
        self.root.rowconfigure(0, weight=1)
        main_frame.columnconfigure(1, weight=1)
        main_frame.rowconfigure(3, weight=1)

        # 标题
        title_label = ttk.Label(main_frame, text="ESP32-S3 AUV 网络监视器",
                                font=('Microsoft YaHei', 18, 'bold'))
        title_label.grid(row=0, column=0, columnspan=2, pady=10)

        # === 串口配置区域 ===
        config_frame = ttk.LabelFrame(main_frame, text="串口配置", padding="10")
        config_frame.grid(row=1, column=0, columnspan=2, sticky=(tk.W, tk.E), pady=5)

        ttk.Label(config_frame, text="串口:", font=('Microsoft YaHei', 10)).grid(row=0, column=0, padx=5)
        self.port_combo = ttk.Combobox(config_frame, width=15, font=('Microsoft YaHei', 10))
        self.port_combo.grid(row=0, column=1, padx=5)
        self.refresh_ports()

        ttk.Button(config_frame, text="刷新", command=self.refresh_ports).grid(row=0, column=2, padx=5)

        ttk.Label(config_frame, text="波特率:", font=('Microsoft YaHei', 10)).grid(row=0, column=3, padx=5)
        self.baudrate_combo = ttk.Combobox(config_frame, width=10, values=['115200', '9600', '57600'],
                                           font=('Microsoft YaHei', 10))
        self.baudrate_combo.set('115200')
        self.baudrate_combo.grid(row=0, column=4, padx=5)

        self.connect_btn = ttk.Button(config_frame, text="连接", command=self.toggle_connection)
        self.connect_btn.grid(row=0, column=5, padx=10)

        # === 左侧：状态显示 + 控制面板 + TCP配置 ===
        left_frame = ttk.Frame(main_frame)
        left_frame.grid(row=2, column=0, sticky=(tk.W, tk.E, tk.N, tk.S), pady=5, padx=(0, 5))
        left_frame.columnconfigure(0, weight=1)

        # 状态显示区域
        status_frame = ttk.LabelFrame(left_frame, text="网络状态", padding="15")
        status_frame.grid(row=0, column=0, sticky=(tk.W, tk.E, tk.N), pady=(0, 10))

        status_items = [
            ("链路状态:", "link_label", "断开", '#ff4444'),
            ("DHCP状态:", "dhcp_label", "等待中", '#ffaa00'),
            ("MAC地址:", "mac_label", "未获取", '#888888'),
            ("IP地址:", "ip_label", "未获取", '#888888'),
            ("子网掩码:", "netmask_label", "未获取", '#888888'),
            ("网关:", "gateway_label", "未获取", '#888888'),
            ("TCP连接:", "tcp_label", "未连接", '#888888'),
            ("继电器:", "relay_label", "OFF", '#888888'),
            ("夹子:", "claw_label", "停止", '#888888'),
            ("连接时长:", "uptime_label", "00:00:00", '#888888'),
        ]

        self.status_refs = {}
        for i, (label_text, ref_name, default_text, default_color) in enumerate(status_items):
            ttk.Label(status_frame, text=label_text, font=('Microsoft YaHei', 11, 'bold')).grid(
                row=i, column=0, sticky=tk.W, pady=5, padx=5)
            label = ttk.Label(status_frame, text=default_text, foreground=default_color,
                              font=('Microsoft YaHei', 11))
            label.grid(row=i, column=1, sticky=tk.W, pady=5, padx=10)
            self.status_refs[ref_name] = label

        # 状态指示灯
        indicator_frame = ttk.Frame(status_frame)
        indicator_frame.grid(row=0, column=2, rowspan=7, padx=20)

        self.indicator_canvas = tk.Canvas(indicator_frame, width=80, height=80, bg='#f0f0f0',
                                          highlightthickness=0)
        self.indicator_canvas.pack()
        self.indicator = self.indicator_canvas.create_oval(10, 10, 70, 70, fill='#ff4444',
                                                           outline='black', width=2)
        self.indicator_text = self.indicator_canvas.create_text(40, 40, text="断开",
                                                                font=('Microsoft YaHei', 10, 'bold'))

        # TCP网络配置面板
        tcp_frame = ttk.LabelFrame(left_frame, text="TCP 网络", padding="10")
        tcp_frame.grid(row=1, column=0, sticky=(tk.W, tk.E, tk.N, tk.S), pady=(0, 10))
        tcp_frame.columnconfigure(1, weight=1)

        # 本地监听端口
        ttk.Label(tcp_frame, text="本地监听端口:").grid(row=0, column=0, sticky=tk.W, padx=5, pady=2)
        self.local_port_entry = ttk.Entry(tcp_frame, width=10)
        self.local_port_entry.grid(row=0, column=1, sticky=tk.W, padx=5, pady=2)
        self.local_port_entry.insert(0, "8080")
        self.tcp_listen_btn = ttk.Button(tcp_frame, text="开始监听", command=self.toggle_tcp_listen)
        self.tcp_listen_btn.grid(row=0, column=2, padx=5, pady=2)

        # 对端连接
        ttk.Label(tcp_frame, text="对端IP:").grid(row=1, column=0, sticky=tk.W, padx=5, pady=2)
        self.peer_ip_entry = ttk.Entry(tcp_frame, width=15)
        self.peer_ip_entry.grid(row=1, column=1, sticky=(tk.W, tk.E), padx=5, pady=2)
        self.peer_ip_entry.insert(0, "192.168.29.100")

        ttk.Label(tcp_frame, text="对端端口:").grid(row=2, column=0, sticky=tk.W, padx=5, pady=2)
        self.peer_port_entry = ttk.Entry(tcp_frame, width=10)
        self.peer_port_entry.grid(row=2, column=1, sticky=tk.W, padx=5, pady=2)
        self.peer_port_entry.insert(0, "8080")

        self.tcp_connect_btn = ttk.Button(tcp_frame, text="连接对端", command=self.toggle_tcp_connect)
        self.tcp_connect_btn.grid(row=2, column=2, padx=5, pady=2)

        # 控制面板
        control_frame = ttk.LabelFrame(left_frame, text="AUV 控制面板", padding="10")
        control_frame.grid(row=2, column=0, sticky=(tk.W, tk.E, tk.N, tk.S))
        control_frame.columnconfigure(1, weight=1)

        ttk.Separator(control_frame, orient='horizontal').grid(row=0, column=0, columnspan=3,
                                                               sticky=(tk.W, tk.E), pady=5)

        # 继电器和夹子控制
        ttk.Label(control_frame, text="继电器:").grid(row=1, column=0, sticky=tk.W, padx=5, pady=5)
        self.relay_var = tk.IntVar(value=0)
        relay_chk = ttk.Checkbutton(control_frame, text="ON", variable=self.relay_var,
                                    command=self.on_control_change)
        relay_chk.grid(row=1, column=1, sticky=tk.W, padx=5, pady=5)

        ttk.Label(control_frame, text="夹子:").grid(row=2, column=0, sticky=tk.W, padx=5, pady=5)
        self.claw_var = tk.IntVar(value=0)
        claw_frame = ttk.Frame(control_frame)
        claw_frame.grid(row=2, column=1, sticky=tk.W, padx=5, pady=5)
        ttk.Radiobutton(claw_frame, text="停止", variable=self.claw_var, value=0,
                        command=self.on_control_change).pack(side=tk.LEFT)
        ttk.Radiobutton(claw_frame, text="正转", variable=self.claw_var, value=1,
                        command=self.on_control_change).pack(side=tk.LEFT)
        ttk.Radiobutton(claw_frame, text="反转", variable=self.claw_var, value=2,
                        command=self.on_control_change).pack(side=tk.LEFT)

        ttk.Button(control_frame, text="发送控制指令", command=self.send_control).grid(
            row=3, column=0, columnspan=3, pady=10)

        # === 右侧：串口日志 + 网络数据 ===
        right_frame = ttk.Frame(main_frame)
        right_frame.grid(row=2, column=1, rowspan=2, sticky=(tk.W, tk.E, tk.N, tk.S), pady=5)
        right_frame.columnconfigure(0, weight=1)
        right_frame.rowconfigure(0, weight=1)
        right_frame.rowconfigure(2, weight=1)

        # 串口日志
        log_frame = ttk.LabelFrame(right_frame, text="串口日志", padding="10")
        log_frame.grid(row=0, column=0, sticky=(tk.W, tk.E, tk.N, tk.S), pady=(0, 5))

        self.log_text = scrolledtext.ScrolledText(log_frame, width=60, height=12,
                                                  font=('Consolas', 9))
        self.log_text.pack(fill=tk.BOTH, expand=True)

        btn_frame = ttk.Frame(log_frame)
        btn_frame.pack(fill=tk.X, pady=5)
        ttk.Button(btn_frame, text="清除日志", command=self.clear_log).pack(side=tk.LEFT, padx=5)
        ttk.Button(btn_frame, text="保存日志", command=self.save_log).pack(side=tk.LEFT, padx=5)

        # 网络数据接收显示
        net_frame = ttk.LabelFrame(right_frame, text="网络数据接收（显示数据来源）", padding="10")
        net_frame.grid(row=2, column=0, sticky=(tk.W, tk.E, tk.N, tk.S))

        self.net_text = scrolledtext.ScrolledText(net_frame, width=60, height=12,
                                                  font=('Consolas', 9))
        self.net_text.pack(fill=tk.BOTH, expand=True)

        net_btn_frame = ttk.Frame(net_frame)
        net_btn_frame.pack(fill=tk.X, pady=5)
        ttk.Button(net_btn_frame, text="清空网络数据", command=self.clear_net_log).pack(side=tk.LEFT, padx=5)
        ttk.Button(net_btn_frame, text="保存网络数据", command=self.save_net_log).pack(side=tk.LEFT, padx=5)

        # === 底部信息 ===
        info_label = ttk.Label(main_frame,
                               text="提示: 绿色=已获取IP  橙色=链路已连接  红色=断开  蓝色=TCP有数据",
                               font=('Microsoft YaHei', 9), foreground='#666666')
        info_label.grid(row=4, column=0, columnspan=2, pady=5)

    # ==================== 串口功能 ====================
    def refresh_ports(self):
        ports = serial.tools.list_ports.comports()
        port_list = [port.device for port in ports]
        self.port_combo['values'] = port_list
        if port_list and not self.port_combo.get():
            self.port_combo.set(port_list[0])

    def toggle_connection(self):
        if self.is_running:
            self.disconnect()
        else:
            self.connect()

    def connect(self):
        port = self.port_combo.get()
        baudrate = int(self.baudrate_combo.get())

        if not port:
            messagebox.showerror("错误", "请选择一个串口")
            return

        try:
            self.serial_port = serial.Serial(port, baudrate, timeout=0.1)
            self.is_running = True
            self.connect_btn.config(text="断开")
            self.log_message(f"✓ 已连接到 {port} @ {baudrate} bps")

            self.serial_thread = threading.Thread(target=self.read_serial_data, daemon=True)
            self.serial_thread.start()

        except Exception as e:
            self.log_message(f"✗ 连接失败: {str(e)}")
            messagebox.showerror("错误", f"无法连接到串口: {str(e)}")

    def disconnect(self):
        self.is_running = False
        if self.serial_port:
            self.serial_port.close()
            self.serial_port = None
        self.connect_btn.config(text="连接")
        self.log_message("✓ 已断开连接")
        self.set_indicator('red', "断开")
        self.link_start_time = None

    def read_serial_data(self):
        while self.is_running and self.serial_port:
            try:
                if self.serial_port.in_waiting > 0:
                    data = self.serial_port.read(self.serial_port.in_waiting)
                    lines = data.decode('utf-8', errors='ignore').split('\n')
                    for line in lines:
                        if line.strip():
                            self.root.after(0, self.process_line, line.strip())
                else:
                    import time
                    time.sleep(0.01)
            except Exception as e:
                if self.is_running:
                    self.root.after(0, self.log_message, f"✗ 读取错误: {str(e)}")
                break

    def process_line(self, line):
        timestamp = datetime.now().strftime("%H:%M:%S")
        self.log_text.insert(tk.END, f"[{timestamp}] {line}\n")
        self.log_text.see(tk.END)

        # 限制日志行数
        if float(self.log_text.index('end-1c').split('.')[0]) > 500:
            self.log_text.delete('1.0', '101.0')

        self.parse_network_info(line)
        self.parse_auv_info(line)
        self.update_uptime()

    # ==================== TCP 服务端 ====================
    def toggle_tcp_listen(self):
        if self.tcp_server_running:
            self.stop_tcp_server()
        else:
            self.start_tcp_server()

    def start_tcp_server(self):
        try:
            port = int(self.local_port_entry.get())
            self.tcp_server_sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.tcp_server_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            self.tcp_server_sock.bind(('0.0.0.0', port))
            self.tcp_server_sock.listen(5)
            self.tcp_server_sock.settimeout(1.0)
            self.tcp_server_running = True

            self.tcp_server_thread = threading.Thread(target=self.tcp_server_loop, daemon=True)
            self.tcp_server_thread.start()

            self.tcp_listen_btn.config(text="停止监听")
            self.log_message(f"TCP 服务端已启动，监听端口 {port}")
        except Exception as e:
            messagebox.showerror("错误", f"启动TCP服务端失败: {e}")

    def stop_tcp_server(self):
        self.tcp_server_running = False
        for sock in list(self.tcp_clients.keys()):
            self.close_tcp_client(sock)
        if self.tcp_server_sock:
            try:
                self.tcp_server_sock.close()
            except Exception:
                pass
            self.tcp_server_sock = None
        self.tcp_listen_btn.config(text="开始监听")
        self.log_message("TCP 服务端已停止")

    def tcp_server_loop(self):
        while self.tcp_server_running:
            try:
                client_sock, client_addr = self.tcp_server_sock.accept()
                client_sock.settimeout(1.0)
                with self.tcp_client_lock:
                    self.tcp_clients[client_sock] = client_addr

                self.root.after(0, lambda a=client_addr: self.net_message(
                    f"[系统] 新客户端连接: {a[0]}:{a[1]}"))
                self.root.after(0, lambda: self.status_refs['tcp_label'].config(
                    text="已连接", foreground='#44aa44'))
                self.root.after(0, lambda: self.set_indicator('blue', "数据"))

                t = threading.Thread(target=self.tcp_client_handler,
                                     args=(client_sock, client_addr), daemon=True)
                t.start()
            except socket.timeout:
                continue
            except Exception as e:
                if self.tcp_server_running:
                    self.root.after(0, lambda e=str(e): self.net_message(f"[系统] 监听异常: {e}"))
                break

    def tcp_client_handler(self, sock, addr):
        buffer = b""
        while self.tcp_server_running:
            try:
                data = sock.recv(1024)
                if not data:
                    break
                buffer += data

                # 按行分割处理
                while b'\n' in buffer:
                    line, buffer = buffer.split(b'\n', 1)
                    text = line.decode('utf-8', errors='ignore').strip()
                    if text:
                        self.root.after(0, lambda t=text, a=addr: self.net_message(
                            f"[{a[0]}:{a[1]}] {t}"))
                        self.root.after(0, lambda t=text: self.parse_tcp_data(t))
            except socket.timeout:
                continue
            except Exception:
                break

        self.close_tcp_client(sock)
        self.root.after(0, lambda a=addr: self.net_message(f"[系统] 客户端断开: {a[0]}:{a[1]}"))

    def close_tcp_client(self, sock):
        with self.tcp_client_lock:
            if sock in self.tcp_clients:
                del self.tcp_clients[sock]
        try:
            sock.close()
        except Exception:
            pass

    # ==================== TCP 客户端 ====================
    def toggle_tcp_connect(self):
        if self.tcp_client_sock:
            self.disconnect_tcp_client()
        else:
            self.connect_tcp_client()

    def connect_tcp_client(self):
        try:
            ip = self.peer_ip_entry.get().strip()
            port = int(self.peer_port_entry.get())
            self.tcp_client_sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.tcp_client_sock.settimeout(5)
            self.tcp_client_sock.connect((ip, port))
            self.tcp_client_sock.settimeout(1.0)
            self.tcp_connect_btn.config(text="断开对端")
            self.log_message(f"TCP 已连接对端 {ip}:{port}")
        except Exception as e:
            self.tcp_client_sock = None
            messagebox.showerror("错误", f"连接对端失败: {e}")

    def disconnect_tcp_client(self):
        if self.tcp_client_sock:
            try:
                self.tcp_client_sock.close()
            except Exception:
                pass
            self.tcp_client_sock = None
        self.tcp_connect_btn.config(text="连接对端")
        self.log_message("TCP 已断开对端")

    def send_tcp(self, data):
        if self.tcp_client_sock:
            try:
                self.tcp_client_sock.sendall(data)
                return True
            except Exception as e:
                self.log_message(f"TCP 发送失败: {e}")
                self.disconnect_tcp_client()
        return False

    # ==================== 数据解析 ====================
    def parse_network_info(self, line):
        if "Ethernet Link Up" in line:
            self.link_status = "已连接"
            self.status_refs['link_label'].config(text="已连接", foreground='#44aa44')
            self.set_indicator('orange', "已连接")
            self.dhcp_status = "等待DHCP..."
            self.status_refs['dhcp_label'].config(text="等待DHCP...", foreground='#ffaa00')
            self.link_start_time = datetime.now()

        elif "Ethernet Link Down" in line:
            self.link_status = "断开"
            self.status_refs['link_label'].config(text="断开", foreground='#ff4444')
            self.set_indicator('red', "断开")
            self.link_start_time = None
            self.dhcp_status = "等待中"
            self.status_refs['dhcp_label'].config(text="等待中", foreground='#ffaa00')

        mac_match = re.search(r'Ethernet HW Addr ([0-9A-Fa-f:]+)', line)
        if mac_match:
            self.mac_address = mac_match.group(1)
            self.status_refs['mac_label'].config(text=self.mac_address, foreground='#4444ff')

        ip_match = re.search(r'ETHIP:\s*(\d+\.\d+\.\d+\.\d+)', line)
        if ip_match:
            self.ip_address = ip_match.group(1)
            self.status_refs['ip_label'].config(text=self.ip_address, foreground='#44aa44')
            self.dhcp_status = "已获取IP"
            self.status_refs['dhcp_label'].config(text="已获取IP", foreground='#44aa44')
            self.set_indicator('green', "在线")
            self.log_message("✓✓✓ DHCP成功获取IP地址!")

        netmask_match = re.search(r'ETHMASK:\s*(\d+\.\d+\.\d+\.\d+)', line)
        if netmask_match:
            self.netmask = netmask_match.group(1)
            self.status_refs['netmask_label'].config(text=self.netmask, foreground='#4444ff')

        gateway_match = re.search(r'ETHGW:\s*(\d+\.\d+\.\d+\.\d+)', line)
        if gateway_match:
            self.gateway = gateway_match.group(1)
            self.status_refs['gateway_label'].config(text=self.gateway, foreground='#4444ff')

        if "Failed to obtain IP address" in line:
            self.dhcp_status = "DHCP超时"
            self.status_refs['dhcp_label'].config(text="DHCP超时", foreground='#ff4444')
            self.log_message("✗ DHCP获取IP超时")

    def parse_auv_info(self, line):
        if "Connected to peer" in line or "New client connected" in line:
            self.tcp_connected = True
            self.status_refs['tcp_label'].config(text="已连接", foreground='#44aa44')
            self.set_indicator('blue', "数据")

        control_match = re.search(r'Control: CH7=(\d+) CH8=(\d+)', line)
        if control_match:
            self.relay = int(control_match.group(1))
            self.claw = int(control_match.group(2))
            self.update_auv_status()

        if '"type":"status"' in line:
            try:
                status = json.loads(line)
                self.relay = status.get('relay', 0)
                self.claw = status.get('claw', 0)
                self.update_auv_status()
            except json.JSONDecodeError:
                pass

    def parse_tcp_data(self, text):
        # 解析通过网络接收到的数据
        if '"type":"status"' in text:
            try:
                status = json.loads(text)
                self.relay = status.get('relay', 0)
                self.claw = status.get('claw', 0)
                self.update_auv_status()
            except json.JSONDecodeError:
                pass

    def update_auv_status(self):
        self.status_refs['relay_label'].config(text="ON" if self.relay else "OFF",
                                               foreground='#44aa44' if self.relay else '#888888')
        claw_text = ["停止", "正转", "反转"][self.claw] if self.claw < 3 else "未知"
        self.status_refs['claw_label'].config(text=claw_text, foreground='#4444ff')

    def update_uptime(self):
        if self.link_start_time:
            elapsed = datetime.now() - self.link_start_time
            hours = int(elapsed.total_seconds() // 3600)
            minutes = int((elapsed.total_seconds() % 3600) // 60)
            seconds = int(elapsed.total_seconds() % 60)
            self.uptime = f"{hours:02d}:{minutes:02d}:{seconds:02d}"
            self.status_refs['uptime_label'].config(text=self.uptime, foreground='#4444ff')

    def set_indicator(self, color, text):
        self.indicator_canvas.itemconfig(self.indicator, fill=color)
        self.indicator_canvas.itemconfig(self.indicator_text, text=text)

    # ==================== 控制指令发送 ====================
    def send_serial(self, data):
        if self.serial_port and self.serial_port.is_open:
            try:
                if isinstance(data, str):
                    data = data.encode('utf-8')
                self.serial_port.write(data)
                self.serial_port.write(b'\r\n')
                self.log_message(f"→ 串口发送: {data.decode('utf-8', errors='ignore').strip()}")
            except Exception as e:
                messagebox.showerror("发送失败", str(e))
        else:
            messagebox.showerror("错误", "串口未连接")

    def send_tcp_command(self, data):
        if self.tcp_client_sock:
            try:
                if isinstance(data, str):
                    data = data.encode('utf-8')
                self.tcp_client_sock.sendall(data + b'\r\n')
                self.net_message(f"[本机发送] {data.decode('utf-8', errors='ignore').strip()}")
            except Exception as e:
                messagebox.showerror("TCP发送失败", str(e))
        else:
            messagebox.showerror("错误", "TCP对端未连接")

    def send_control(self):
        ch7 = self.relay_var.get()
        ch8 = self.claw_var.get()

        sentence = f"$AUVC,0.000,0.000,{ch7},{ch8}"
        checksum = 0
        for ch in sentence[1:]:
            checksum ^= ord(ch)
        nmea = f"{sentence}*{checksum:02X}"

        # 优先通过TCP发送，否则通过串口
        if self.tcp_client_sock:
            self.send_tcp_command(nmea)
        else:
            self.send_serial(nmea)

    def on_control_change(self, event=None):
        pass

    # ==================== 日志功能 ====================
    def log_message(self, message):
        timestamp = datetime.now().strftime("%H:%M:%S")
        self.log_text.insert(tk.END, f"[{timestamp}] {message}\n")
        self.log_text.see(tk.END)

    def net_message(self, message):
        timestamp = datetime.now().strftime("%H:%M:%S")
        self.net_text.insert(tk.END, f"[{timestamp}] {message}\n")
        self.net_text.see(tk.END)

        # 限制行数
        if float(self.net_text.index('end-1c').split('.')[0]) > 500:
            self.net_text.delete('1.0', '101.0')

    def clear_log(self):
        self.log_text.delete(1.0, tk.END)

    def clear_net_log(self):
        self.net_text.delete(1.0, tk.END)

    def save_log(self):
        filename = filedialog.asksaveasfilename(
            defaultextension=".txt",
            filetypes=[("文本文件", "*.txt"), ("所有文件", "*.*")]
        )
        if filename:
            with open(filename, 'w', encoding='utf-8') as f:
                f.write(self.log_text.get(1.0, tk.END))
            self.log_message(f"✓ 日志已保存到: {filename}")

    def save_net_log(self):
        filename = filedialog.asksaveasfilename(
            defaultextension=".txt",
            filetypes=[("文本文件", "*.txt"), ("所有文件", "*.*")]
        )
        if filename:
            with open(filename, 'w', encoding='utf-8') as f:
                f.write(self.net_text.get(1.0, tk.END))
            self.net_message(f"[系统] 网络数据已保存到: {filename}")

    def on_closing(self):
        self.stop_tcp_server()
        self.disconnect_tcp_client()
        self.disconnect()
        self.root.destroy()


def main():
    root = tk.Tk()
    app = ESP32NetworkMonitor(root)
    root.protocol("WM_DELETE_WINDOW", app.on_closing)
    root.mainloop()


if __name__ == "__main__":
    main()
