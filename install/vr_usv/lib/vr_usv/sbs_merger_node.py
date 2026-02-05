#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import message_filters
import numpy as np

class SBSStitcher(Node):

    def __init__(self):
        super().__init__('sbs_stitcher')
        
        self.bridge = CvBridge()

        #  Suscriptores con filtro de tiempo
      
        self.left_sub = message_filters.Subscriber(self, Image, 'left_image')
        self.right_sub = message_filters.Subscriber(self, Image, 'right_image')

        # Sincronizador 
        self.ts = message_filters.ApproximateTimeSynchronizer(
            [self.left_sub, self.right_sub], 
            queue_size=10, #numero de frames que se pueden almacenar para la sincronización
            slop=0.05 # tolerancia de tiempo para considerar dos frames como sincronizados (en segundos)
        )
        self.ts.registerCallback(self.stitch_callback)

        # 3. Publicador de la imagen 
        self.publisher = self.create_publisher(Image, 'output_image', 10)
        
        self.get_logger().info('Nodo SBS Stitcher iniciado y esperando imágenes...')

    def stitch_callback(self, left_msg, right_msg):
        try:
            # Convertir mensajes de ROS 2 a formato OpenCV
            cv_left = self.bridge.imgmsg_to_cv2(left_msg, desired_encoding='bgr8')
            cv_right = self.bridge.imgmsg_to_cv2(right_msg, desired_encoding='bgr8')

            # --- PROCESAMIENTO OPCIONAL ---
            # Si las cámaras tienen resoluciones distintas, redimensionarlas aquí
            # height = 480
            # width = 640
            # cv_left = cv2.resize(cv_left, (width, height))
            # cv_right = cv2.resize(cv_right, (width, height))

            # --- UNIÓN SBS (Side-by-Side) ---
            # Concatenación horizontal: pega la derecha al lado de la izquierda
            sbs_img = cv2.hconcat([cv_left, cv_right])

            # Convertir de vuelta a mensaje de ROS 2
            out_msg = self.bridge.cv2_to_imgmsg(sbs_img, encoding='bgr8')
            out_msg.header = left_msg.header # Mantener el timestamp original
            
            self.publisher.publish(out_msg)

        except Exception as e:
            self.get_logger().error(f'Error procesando imágenes: {str(e)}')

def main(args=None):
    rclpy.init(args=args)
    node = SBSStitcher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()