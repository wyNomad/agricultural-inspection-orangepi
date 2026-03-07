#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Quaternion
from nav_msgs.msg import Odometry
import serial
import struct
import math
import tf_transformations # 确保安装: sudo apt install ros-$ROS_DISTRO-tf-transformations

class ChassisFullNode(Node):
    def __init__(self):
        super().__init__('chassis_node')
        
        # --- 1. 声明参数 ---
        self.declare_parameter('port', '/dev/ttyACM0')
        self.declare_parameter('baudrate', 115200)
        self.declare_parameter('L', 0.11)  # ⚠️ 修改为第二份代码中的默认值或你的实际值

        self.publish_count = 0  # 用于分频发布的计数器

        self.port = self.get_parameter('port').value
        self.baud = self.get_parameter('baudrate').value
        self.L = self.get_parameter('L').value

        # --- 2. 硬件连接 ---
        self.ser = None
        try:
            self.ser = serial.Serial(self.port, self.baud, timeout=0.05)
            self.get_logger().info(f'✅ 底盘串口已连接: {self.port} ({self.baud})')
        except Exception as e:
            self.get_logger().error(f'❌ 串口连接失败: {e}')
            # 注意：连接失败我们不退出，防止 ROS 节点直接崩溃，方便后续重连调试
            
        # --- 3. ROS 通信初始化 ---
        
        # [订阅] 速度指令 -> 发送到底盘
        self.cmd_sub = self.create_subscription(Twist, 'cmd_vel', self.cmd_callback, 10)
        
        # [发布] 里程计数据 -> 供 EKF 使用
        self.odom_pub = self.create_publisher(Odometry, 'wheel/odom', 10)

        # --- 4. 里程计状态变量 ---
        self.x = 0.0
        self.y = 0.0
        self.th = 0.0
        self.last_time = self.get_clock().now()

        # [定时器] 50Hz 频率读取串口回传 (RX)
        self.create_timer(0.02, self.receive_callback)

    # ================= 接收逻辑 =================
    def receive_callback(self):
        if self.ser is None or not self.ser.is_open:
            return

        # --- 修改点 1: 使用 while 循环，确保一次性处理完缓冲区里的所有堆积帧 ---
        while self.ser.in_waiting >= 15:
            # 1. 寻找帧头 0x55
            header1 = self.ser.read(1)
            if header1 != b'\x55':
                continue # 如果不是 0x55，继续找下一个字节，直到缓冲区读空或找到为止

            # 2. 确认第二个帧头 0xAA
            header2 = self.ser.read(1)
            if header2 != b'\xAA':
                # 如果第一个是对的但第二个不对，说明这也不是我们要的包，继续找
                continue

            # 3. 读取载荷 (12字节数据 + 1字节校验)
            payload = self.ser.read(13)
            if len(payload) < 13:
                break # 缓冲区数据不够了，跳出循环等下次

            # 4. 校验
            calc_checksum = 0
            for b in payload[:12]:
                calc_checksum ^= b
            
            if calc_checksum != payload[12]:
                self.get_logger().error(f"校验失败! 丢弃该包")
                continue # 校验失败，寻找下一帧

            # 5. 解析并更新
            try:
                va, vb, vc = struct.unpack('<3f', payload[:12])
                self.update_odom(va, vb, vc)    # 调用update_odom即发出话题
            except Exception as e:
                self.get_logger().error(f"解析异常: {e}")

        # --- 修改点 2: 预防性清理 (可选) ---
        # 如果缓冲区堆积过大（比如超过 100 字节），说明处理速度严重跟不上了
        if self.ser.in_waiting > 100:
            self.get_logger().warn("缓冲区堆积严重，执行强制清空！")
            self.ser.reset_input_buffer()

    # def receive_callback(self):
    #     # 串口未初始化/未打开，直接返回
    #     if self.ser is None or not self.ser.is_open:
    #         return

    #     # 缓冲区字节数不足15字节（帧头2 + 数据12 + 校验1），不解析，避免读空
    #     if self.ser.in_waiting < 15:
    #         return
        
    #     # 1. 寻找帧头 0x55 0xAA
    #     # 这是一个严格的检查，如果错位，可能一直进不来
    #     header1 = self.ser.read(1)
    #     if header1 != b'\x55': 
    #         # self.get_logger().warn(f"错位! 期望55, 实际: {header1.hex()}") 
    #         return # 帧头1不对，丢弃
        
    #     header2 = self.ser.read(1)
    #     if header2 != b'\xAA': 
    #         # self.get_logger().warn(f"错位! 期望AA, 实际: {header2.hex()}")
    #         return # 帧头2不对，丢弃

    #     # 2. 读取载荷
    #     payload = self.ser.read(13)
    #     if len(payload) < 13: return # 没读到13字节，说明数据不完整，退出

    #     # 3. 校验
    #     calc_checksum = 0
    #     for b in payload[:12]:
    #         calc_checksum ^= b
        
    #     # --- 步骤 2: 如果校验挂了，打印出来 ---
    #     if calc_checksum != payload[12]:
    #         self.get_logger().error(
    #             f"校验失败! 算出来是: {hex(calc_checksum)}, 收到的最后一位是: {hex(payload[12])}"
    #         )
    #         return

    #     # 4. 解析数据
    #     try:
    #         va, vb, vc = struct.unpack('<3f', payload[:12])
            
    #         # --- 步骤 3: 这就是你想要的！直接打印三个轮子的原始速度 ---
    #         # self.get_logger().info(
    #         #    f"✅ 原始轮速 -> va:{va:.3f} vb:{vb:.3f} vc:{vc:.3f}", 
    #         #    throttle_duration_sec=0.2
    #         #)
            
    #         self.update_odom(va, vb, vc)
            
    #     except Exception as e:
    #         self.get_logger().error(f"解析炸了: {e}")

    def update_odom(self, va, vb, vc):
        """里程计运动学解算"""
        now = self.get_clock().now()
        dt = (now - self.last_time).nanoseconds / 1e9
        
        # [优化] dt 保护：防止串口卡顿导致的巨大积分跳变
        if dt <= 0 or dt > 0.1: 
            dt = 0.02
        
        self.last_time = now

        # --- 运动学正解 ---
        # vx: 前进, vy: 左移, vth: 逆时针旋转
        vx = (math.sqrt(3) / 3.0) * (vc - vb)
        vy = (-2.0 * va + vb + vc) / 3.0
        vth = (va + vb + vc) / (3.0 * self.L)

        # --- 位姿积分 ---
        # 将车身速度投影到世界坐标系
        delta_x = (vx * math.cos(self.th) - vy * math.sin(self.th)) * dt
        delta_y = (vx * math.sin(self.th) + vy * math.cos(self.th)) * dt
        delta_th = vth * dt

        self.x += delta_x
        self.y += delta_y
        self.th += delta_th
        
        # [优化] 角度归一化：保持在 [-pi, pi] 之间
        self.th = math.atan2(math.sin(self.th), math.cos(self.th))

        # --- 组装消息 ---
        odom = Odometry()
        odom.header.stamp = now.to_msg()
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'

        # 填充速度 (Twist)
        odom.twist.twist.linear.x = vx
        odom.twist.twist.linear.y = vy
        odom.twist.twist.angular.z = vth

        # 填充位置 (Pose)
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        
        # 欧拉角转四元数
        q = tf_transformations.quaternion_from_euler(0, 0, self.th)
        odom.pose.pose.orientation = Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])
        
        # --- 协方差矩阵 ---
        # 1. Pose Covariance (位置不确定性)
        # 这个其实 EKF 不怎么看，因为我们只融合速度，但填个小数值是为了让 RViz 好看点
        p_cov = [0.0] * 36
        p_cov[0] = 1e-5   # X: 0.00001
        p_cov[7] = 1e-5   # Y: 0.00001
        p_cov[35] = 5e-2  # Yaw: 0.05 (转角稍微给大一点点没事)
        # 补全其他对角线，防止矩阵奇异（可选，但推荐）
        p_cov[14] = 1e-5 # Z
        p_cov[21] = 1e-5 # Roll
        p_cov[28] = 1e-5 # Pitch
        odom.pose.covariance = p_cov

        # 2. Twist Covariance (速度不确定性) - [关键！] EKF 实际上用的是这个！
        t_cov = [0.0] * 36
        t_cov[0] = 1e-5   # vX: 非常信任前进速度
        t_cov[7] = 1e-5   # vY: 非常信任横移速度
        t_cov[35] = 1e-4  # vYaw: 稍微给大一点容忍打滑，但绝不能是 0.1
        # 补全其他对角线
        t_cov[14] = 1e-5
        t_cov[21] = 1e-5
        t_cov[28] = 1e-5
        odom.twist.covariance = t_cov

        # --- 2. 分频发布逻辑 (25Hz 逻辑) ---
        self.publish_count += 1
        if self.publish_count >= 2:  # 每 2 次计算发布一次
            self.odom_pub.publish(odom)
            self.publish_count = 0

        # 调试输出
        self.get_logger().info(
            f"ODOM -> Pose:[{self.x:.2f}, {self.y:.2f}, {math.degrees(self.th):.1f}°] | Vel:[{vx:.2f}, {vy:.2f}, {vth:.2f}]", 
            throttle_duration_sec=0.5
        )
        
    # ================= 发送逻辑 (TX - 保持不变) =================
    def cmd_callback(self, msg):
        if self.ser is None or not self.ser.is_open:
            return

        # 1. 提取数据
        vx = float(msg.linear.x)
        vy = float(msg.linear.y) 
        wz = float(msg.angular.z)

        # 2. 打包协议 (Header: 0xAA 0x55)
        header = b'\xAA\x55'
        payload = struct.pack('<fff', vx, vy, wz)
        
        # 3. 校验 (TX 校验逻辑保持原样：仅 Payload 异或)
        checksum = 0
        for byte in payload:
            checksum ^= byte
        
        frame = header + payload + bytes([checksum])

        # 4. 发送
        try:
            self.ser.write(frame)
        except Exception as e:
            self.get_logger().error(f'串口写入错误: {e}')

    def stop_robot(self):
        """安全停车指令"""
        if self.ser and self.ser.is_open:
            try:
                # 发送全 0 速度
                stop_frame = b'\xAA\x55' + struct.pack('<fff', 0.0, 0.0, 0.0) + b'\x00'
                self.ser.write(stop_frame)
                self.get_logger().info("🛑 已发送停车指令")
            except:
                pass

def main(args=None):
    rclpy.init(args=args)
    node = ChassisFullNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop_robot()
        if node.ser:
            node.ser.close()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()