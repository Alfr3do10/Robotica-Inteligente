#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge
import cv2 as cv
import numpy as np

class ColorDetectionNode(Node):
    def __init__(self):
        super().__init__('color_detection_node')
        self.wait_for_ros_time()  # Espera a que ROS time esté activo

        # Conversión ROS↔OpenCV
        self.bridge = CvBridge()
        self.img = None

        # Rangos HSV para rojo (dos rangos), verde y amarillo
        self.red_lower1   = np.array([  0, 100, 100], np.uint8)
        self.red_upper1   = np.array([ 10, 255, 255], np.uint8)
        self.red_lower2   = np.array([160, 100, 100], np.uint8)
        self.red_upper2   = np.array([179, 255, 255], np.uint8)

        self.green_lower = np.array([45, 80, 120], np.uint8)
        self.green_upper = np.array([75, 250, 255], np.uint8)

        self.yellow_lower = np.array([25, 15, 140], np.uint8)
        self.yellow_upper = np.array([45, 250, 255], np.uint8)

        # Subscripción a cámara cruda (topic: '/video_source/raw')
        self.subscription = self.create_subscription(
            Image, '/video_source/raw', self.camera_callback, 10)

        # Publicador del estado del semáforo (topic: '/detected_color')
        self.color_pub = self.create_publisher(
            String, 'detected_color', 10)

        # Para recordar el color previo y solo publicar al cambiar
        self.last_color = None

        # Timer de 10 Hz para procesar cada frame
        self.create_timer(0.1, self.control_loop)
        self.get_logger().info('Nodo de detección de color iniciado.')

    def wait_for_ros_time(self):
        self.get_logger().info('Esperando a ROS time...')
        while rclpy.ok():
            if self.get_clock().now().nanoseconds > 0:
                self.get_logger().info('¡ROS time activa!')
                return
            rclpy.spin_once(self, timeout_sec=0.1)

    def camera_callback(self, msg: Image):
        try:
            self.img = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        except Exception as e:
            self.get_logger().error(f'Error al convertir imagen: {e}')

    def control_loop(self):
        if self.img is None:
            return

        # 1) Suavizado y conversión a HSV
        blurred = cv.GaussianBlur(self.img, (9, 9), 2)
        hsv     = cv.cvtColor(blurred, cv.COLOR_BGR2HSV)

        # 2) Umbrales por color
        r1      = cv.inRange(hsv, self.red_lower1, self.red_upper1)
        r2      = cv.inRange(hsv, self.red_lower2, self.red_upper2)
        red     = cv.bitwise_or(r1, r2)
        green   = cv.inRange(hsv, self.green_lower, self.green_upper)
        yellow  = cv.inRange(hsv, self.yellow_lower, self.yellow_upper)

        # 3) Limpieza de ruido
        kernel = np.ones((3,3), np.uint8)
        red    = cv.dilate(cv.erode(red,    kernel, iterations=8), kernel, iterations=8)
        green  = cv.dilate(cv.erode(green,  kernel, iterations=8), kernel, iterations=8)
        yellow = cv.dilate(cv.erode(yellow, kernel, iterations=8), kernel, iterations=8)

        # 4) Detección con prioridad: rojo > amarillo > verde
        if   np.count_nonzero(red)    > 0:
            color = 'red'
        elif np.count_nonzero(yellow) > 0:
            color = 'yellow'
        elif np.count_nonzero(green)  > 0:
            color = 'green'
        else:
            color = None

        # 5) Solo publicar cuando cambia el color detectado
        if color != self.last_color:
            self.last_color = color
            msg = String()
            msg.data = color if color else 'none'
            self.color_pub.publish(msg)
            # Mensaje en terminal una sola vez por cambio de estado
            if   color == 'red':
                self.get_logger().info('🔴 Detected RED — detener robot')
            elif color == 'yellow':
                self.get_logger().info('🟡 Detected YELLOW — desacelerar robot')
            elif color == 'green':
                self.get_logger().info('🟢 Detected GREEN — acelerar robot')
            else:
                self.get_logger().info('⚪ Sin semáforo visible')

def main(args=None):
    """Función principal, ahora definida a nivel de módulo."""
    rclpy.init(args=args)
    node = ColorDetectionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
