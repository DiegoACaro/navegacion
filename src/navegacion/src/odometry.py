#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, TransformStamped, Quaternion
import tf2_ros 
import tf2_geometry_msgs 
import math
import time
import numpy as np

class OdometryPos(Node):
    def __init__(self):
        super().__init__('odometry_pos')
        self.cmd_vel_publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        self.subscription = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10
        )

    
        
    # Solicitar al usuario los valores de posición deseados
        self.target_x = float(input("Ingrese el valor deseado para x: "))
        self.target_y = float(input("Ingrese el valor deseado para y: "))
        #self.target_w = (float(input("ingrese angulo rad: "))*math.pi)/180
        self.integral_lineal = 0
        self.integral_angular = 0
        self.timer_ini = time.time() 

    #Variables del filtro
        self.values = np.zeros(5)
        self.a = 0 #variable de contorl
    #funcion del filtro de mediana
    def filtro_media(self, new_x):
        self.values[self.a] = new_x
        self.a += 1
        if self.a>=len(self.values-1):
            self.a=0
        sorted_values = np.sort(self.values)
        return sorted_values[2]

        

    def euler_from_quaternion(self, orientation):
        """
        Convert a quaternion into euler angles (roll, pitch, yaw)
        roll is rotation around x in radians (counterclockwise)
        pitch is rotation around y in radians (counterclockwise)
        yaw is rotation around z in radians (counterclockwise)
        """
     
        t0 = +2.0 * (orientation.w * orientation.x + orientation.y * orientation.z)
        t1 = +1.0 - 2.0 * (orientation.x * orientation.x + orientation.y * orientation.x)
        #roll_x = math.atan2(t0, t1)
     
        t2 = +2.0 * (orientation.w * orientation.y - orientation.z * orientation.x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        #pitch_y = math.asin(t2)
     
        t3 = +2.0 * (orientation.w * orientation.z + orientation.x * orientation.y)
        t4 = +1.0 - 2.0 * (orientation.y * orientation.y + orientation.z * orientation.z)
        yaw_z = math.atan2(t3, t4)
        if yaw_z<0:
            yaw_z += 2*math.pi
     
        return yaw_z # in radians
       
    
    def odom_callback(self, msg):
        # Extract quaternion from TransformStamped message
        #elapsed_time = time.time() - self.timer_ini
          # Obtener la posición actual del TurtleBot3 desde el mensaje de odometría
        angulo_target = math.atan2(self.target_x, self.target_y) * (180 / math.pi)
        if angulo_target < 0:
            angulo_target += 360
        


        current_x = msg.pose.pose.position.x
        current_y = msg.pose.pose.position.y

        yaw_z = self.euler_from_quaternion(msg.pose.pose.orientation)
        ang_filtrado =self.filtro_media(yaw_z) #invocacion del filtro 
        ang_filtrado  *= (180 / math.pi)
        if ang_filtrado < 0:
            ang_filtrado += 360
                



        if(abs(ang_filtrado - angulo_target) < (360 - abs(ang_filtrado - angulo_target))):
            moverse_grados= abs(ang_filtrado-angulo_target)
        else:
            moverse_grados = (360 - abs(ang_filtrado - angulo_target))
        # ----------------------------

        # -----eleccion de direccion--
        direccion = 0
        if (ang_filtrado<angulo_target):
            if((abs(ang_filtrado-angulo_target))<180):
                
                direccion = 1
                
            else:
                
                direccion = -1
                
        elif((abs(ang_filtrado-angulo_target))<180):
            
            direccion = -1
            
        else:
            
            direccion = 1
            

        



        error = abs(angulo_target-ang_filtrado)
        
        linear_velocity = 0.0
        angular_velocity = 0.2 * direccion

        self.get_logger().info(f"angulo_target: {angulo_target}") 

        print("error: ", error)

        #self.get_logger().info(f"angulo: {(ang_filtrado*180)/math.pi}") #Imprimir el angulo actual 

        if error< 0.05:
            linear_velocity = 0.0
            angular_velocity = 0.0
        # Publicar el comando de velocidad
        #if (abs(self.target_x-current_x<=0.1) and abs(self.target_y-current_y)<=0.1):
       

       # self.get_logger().info("Roll: {}, Pitch: {}, Yaw: {}".format(euler[0], euler[1], euler[2]))

        cmd_vel_msg = Twist()
        cmd_vel_msg.linear.x = linear_velocity
        cmd_vel_msg.angular.z = angular_velocity
        self.cmd_vel_publisher.publish(cmd_vel_msg)
        self.last_time = time.time()

def main(args=None):
    rclpy.init(args=args)
    mi_nodo = OdometryPos()
    rclpy.spin(mi_nodo)
    mi_nodo.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
