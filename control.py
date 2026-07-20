import socket
import time

def connect_esp32(ip, port=8080):
    """连接到ESP32-S3"""
    try:
        s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        s.connect((ip, port))
        s.settimeout(5)
        print(f"已连接到 ESP32-S3: {ip}:{port}")
        return s
    except Exception as e:
        print(f"连接失败: {e}")
        return None

def send_command(sock, cmd):
    """发送命令并等待响应"""
    try:
        sock.sendall(cmd.encode())
        time.sleep(0.1)
        # 尝试接收响应
        try:
            response = sock.recv(1024).decode().strip()
            if response:
                print(f"响应: {response}")
        except socket.timeout:
            pass
        return True
    except Exception as e:
        print(f"发送失败: {e}")
        return False

def control_claw(sock, action):
    """控制夹子动作"""
    actions = {
        'clamp': '$AUVC,0.0,0.0,0,1*00\r\n',   # 夹紧
        'release': '$AUVC,0.0,0.0,0,2*00\r\n', # 松开
        'stop': '$AUVC,0.0,0.0,0,0*00\r\n'     # 停止
    }
    if action in actions:
        print(f"执行: {action}")
        send_command(sock, actions[action])
    else:
        print(f"未知动作: {action}")

def control_relay(sock, state):
    """控制继电器"""
    cmd = f'$AUVC,0.0,0.0,{1 if state else 0},0*00\r\n'
    print(f"继电器: {'开启' if state else '关闭'}")
    send_command(sock, cmd)

def main():
    # ESP32-S3 的IP地址（需要根据实际情况修改）
    esp32_ip = "192.168.29.38"
    
    print("=" * 50)
    print(" ESP32-S3 AUV 控制器")
    print("=" * 50)
    print(f"目标IP: {esp32_ip}:8080")
    print()
    
    # 连接ESP32
    sock = connect_esp32(esp32_ip)
    if not sock:
        print("无法连接到ESP32-S3，请检查网络和IP地址")
        return
    
    print()
    print("命令列表:")
    print("  c / clamp  - 夹紧夹子")
    print("  r / release - 松开夹子")
    print("  s / stop   - 停止夹子")
    print("  on         - 开启继电器")
    print("  off        - 关闭继电器")
    print("  quit / exit - 退出")
    print()
    
    try:
        while True:
            cmd = input("输入命令: ").strip().lower()
            
            if cmd in ['quit', 'exit', 'q']:
                print("退出程序")
                break
            elif cmd in ['c', 'clamp']:
                control_claw(sock, 'clamp')
            elif cmd in ['r', 'release']:
                control_claw(sock, 'release')
            elif cmd in ['s', 'stop']:
                control_claw(sock, 'stop')
            elif cmd == 'on':
                control_relay(sock, True)
            elif cmd == 'off':
                control_relay(sock, False)
            else:
                print("未知命令，请输入 c/r/s/on/off/quit")
            
            time.sleep(0.2)
            
    except KeyboardInterrupt:
        print("\n程序被中断")
    finally:
        sock.close()
        print("连接已关闭")

if __name__ == "__main__":
    main()
