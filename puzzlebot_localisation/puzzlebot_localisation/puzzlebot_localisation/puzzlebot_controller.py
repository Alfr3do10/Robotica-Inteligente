#!/usr/bin/env python3
import rclpy
import numpy as np
import signal
from rclpy.node import Node
from rclpy import qos
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped, Twist
from std_msgs.msg import String

class Controller(Node):
    def __init__(self):
        super().__init__('controller')

        # —————— PID distancia ——————
        self.Kp_d, self.Ki_d, self.Kd_d = 1.6, 0.001, 0.05
        # —————— PID ángulo ——————
        self.Kp_th, self.Ki_th, self.Kd_th = 1.0, 0.012, 0.013

        # Integrales y errores previos
        self.e_d_int = self.e_th_int = 0.0
        self.prev_e_d = self.prev_e_th = 0.0

        # Límites de velocidad
        self.max_v = 0.29
        self.max_w = 0.5
        self.min_v = 0.05
        self.min_w = 0.05

        # Pose actual y objetivo
        self.x = self.y = self.theta = 0.0
        self.gx = self.gy = None

        # —————— ESTADO DE ACTIVACIÓN ——————
        # Se inicia detenido hasta recibir el primer VERDE
        self.active = False
        # Flag para omitir rampa en ese primer VERDE
        self.skip_ramp_green = False

        # —————— Lógica de semáforo ——————
        self.color_state      = None
        self.state_start_time = None
        self.slow_factor      = 0.3   # velocidad en AMARILLO = 30% del nominal
        self.ramp_duration    = 2.0   # segundos para rampa de desacel/recup

        # Subscripciones
        self.sub_odom = self.create_subscription(
            Odometry, 'corrected_odom', self.odom_cb,
            qos.qos_profile_sensor_data)
        self.sub_goal = self.create_subscription(
            PoseStamped, 'goal_pose', self.goal_cb,
            qos.qos_profile_sensor_data)
        self.sub_color = self.create_subscription(
            String, 'detected_color', self.color_cb, 10)

        # Publicador de cmd_vel
        self.pub_cmd = self.create_publisher(Twist, 'cmd_vel', 10)

        # Timer de control a 20 Hz
        self.create_timer(0.05, self.control_loop)
        self.get_logger().info('Controller iniciado (detenido hasta primer VERDE).')

    def odom_cb(self, msg: Odometry):
        # Actualizamos pose a partir de odometría corregida
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        siny = 2 * (q.w*q.z + q.x*q.y)
        cosy = 1 - 2 * (q.y*q.y + q.z*q.z)
        self.theta = np.arctan2(siny, cosy)

    def goal_cb(self, msg: PoseStamped):
        # Nuevo objetivo → reset PID
        self.gx = msg.pose.position.x
        self.gy = msg.pose.position.y
        self.e_d_int = self.e_th_int = 0.0
        self.prev_e_d = self.prev_e_th = 0.0

    def color_cb(self, msg: String):
        prev = self.color_state
        # 'none' → None, sino el color detectado
        self.color_state = None if msg.data == 'none' else msg.data
        now = self.get_clock().now()
        self.state_start_time = now

        # —————— AL PRIMER VERDE, ACTIVAR MOVIMIENTO y omitir rampa ——————
        if not self.active and self.color_state == 'green':
            self.active = True
            self.skip_ramp_green = True
            self.get_logger().info('🟢 PRIMER VERDE DETECTADO → MOVIMIENTO ACTIVADO')

        # Mensajes por cambio de color
        if self.color_state != prev:
            if   self.color_state == 'red':
                self.get_logger().info('🔴 SEMÁFORO ROJO → DETENER')
            elif self.color_state == 'yellow':
                self.get_logger().info('🟡 SEMÁFORO AMARILLO → DESACELERAR')
            elif self.color_state == 'green':
                self.get_logger().info('🟢 SEMÁFORO VERDE → RECUPERAR VELOCIDAD')

    def control_loop(self):
        # —————— Si no hemos visto primer VERDE, seguimos detenidos ——————
        if not self.active:
            twist = Twist()
            self.pub_cmd.publish(twist)
            return

        # —————— Si no hay objetivo, frenar ——————
        if self.gx is None:
            twist = Twist()
            self.pub_cmd.publish(twist)
            return

        # —————— Cálculo de errores ——————
        ex = self.gx - self.x
        ey = self.gy - self.y
        e_d = np.hypot(ex, ey)
        theta_d = np.arctan2(ey, ex)
        e_th = (theta_d - self.theta + np.pi) % (2*np.pi) - np.pi
        dt = 0.05

        # — PID angular —
        self.e_th_int += e_th * dt
        de_th = (e_th - self.prev_e_th) / dt
        w_cmd = (self.Kp_th*e_th
                 + self.Ki_th*self.e_th_int
                 + self.Kd_th*de_th)
        self.prev_e_th = e_th

        # — PID distancia —
        self.e_d_int += e_d * dt
        self.e_d_int = np.clip(self.e_d_int, -0.5, 0.5)
        de_d = (e_d - self.prev_e_d) / dt
        v_cmd = (self.Kp_d*e_d
                 + self.Ki_d*self.e_d_int
                 + self.Kd_d*de_d)
        self.prev_e_d = e_d

        # — Evitar saturación cruzada —
        v_cmd *= np.exp(-5 * abs(e_th))

        # — Fase de giro pura: si desviación >10°, solo giramos —
        if abs(e_th) > np.deg2rad(10):
            v_cmd = 0.0

        # — Zona muerta angular refinada —
        if abs(w_cmd) < self.min_w and abs(e_th) > np.deg2rad(2):
            w_cmd = np.sign(w_cmd) * self.min_w

        # — Semáforo: rampas de desacel/recup —
        # Si skip_ramp_green está activo y estamos en verde, saltamos rampa
        if self.skip_ramp_green and self.color_state == 'green':
            # velocidad nominal completa
            v_after = v_cmd
            # desactivamos el skip para el siguiente ciclo
            self.skip_ramp_green = False
        else:
            now     = self.get_clock().now()
            elapsed = ((now - self.state_start_time).nanoseconds * 1e-9) if self.state_start_time else 0.0

            if   self.color_state == 'red':
                v_after = 0.0
            elif self.color_state == 'yellow':
                frac   = min(elapsed, self.ramp_duration) / self.ramp_duration
                factor = 1.0 - frac * (1.0 - self.slow_factor)
                v_after = v_cmd * factor
            elif self.color_state == 'green':
                frac   = min(elapsed, self.ramp_duration) / self.ramp_duration
                # rampa de slow_factor → 1.0
                factor = self.slow_factor + frac * (1.0 - self.slow_factor)
                v_after = v_cmd * factor
            else:
                # sin semáforo
                v_after = v_cmd

        # — Zona muerta y saturación final —
        if abs(v_after) < self.min_v:
            v_after = 0.0
        v_after = float(np.clip(v_after, 0.0, self.max_v))
        w_cmd  = float(np.clip(w_cmd, -self.max_w, self.max_w))

        # — Publicar cmd_vel —
        twist = Twist()
        twist.linear.x  = v_after
        twist.angular.z = w_cmd
        self.pub_cmd.publish(twist)

    def stop_handler(self, sig, frame):
        self.get_logger().info('Controller detenido.')
        raise SystemExit

def main():
    rclpy.init()
    node = Controller()
    signal.signal(signal.SIGINT, node.stop_handler)
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
