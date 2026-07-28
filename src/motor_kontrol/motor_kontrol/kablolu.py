import rclpy
from rclpy.node import Node
import serial
import serial.tools.list_ports
import time

from rover_msgs.msg import ControllerMsg
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy



class SerialJoystickPublisher(Node):

    def __init__(self):
        super().__init__("serial_joystick_publisher")

        # ROS publisher
        self.publisher_ = self.create_publisher(
            ControllerMsg,
            "joystick_cmd",
            10
        )
        
        self.otonom_sub = self.create_subscription( Twist,"cmd_vel",self.otonom_callback, 20)
        
        self.joy_sub = self.create_subscription(
         Joy,
        "/joy",
        self.joy_callback,
        20,
        )

        self.joy_msg = ControllerMsg()
        self.mode = 0
        self.kp = 0.0
        self.kd = 0.0
        self.ki = 0.0
        
        self.otonom_linear_x  = 0.0
        self.otonom_linear_y  = 0.0
        self.otonom_angular_x = 0.0
        self.otonom_angular_y = 0.0
        self.otonom_angular_z = 0.0
        self.last_otonom_time = self.get_clock().now()
        
        self.yonelim_filtered  = 0.0
        self.throttle_filtered = 0.0
        self.filter_alpha      = 0.4
        

 # ── Wheel speed smoothing ─────────────────────────────
        self.sag_hiz_cmd      = 0.0
        self.sol_hiz_cmd      = 0.0
        self.max_speed_change = 10.0
        self.previous_base = 0.0


    def otonom_callback(self, msg: Twist):
        self.otonom_linear_x  = msg.linear.x
        self.otonom_linear_y  = msg.linear.y
        self.otonom_angular_x = msg.angular.x
        self.otonom_angular_y = msg.angular.y
        self.otonom_angular_z = msg.angular.z
        self.last_otonom_time = self.get_clock().now()
        
    def clamp_wheel_speed(self, value: float) -> float:
        if value == 0.0:
            return 0.0
        elif 0.0 < value < 0.35:
            return 0.75
        elif -0.35 < value < 0.0:
            return -0.75
        elif value > 0.6:
            return 1.0
        elif value < -0.6:
            return -1.0
        return value
    
    
    def low_pass_filter(self, current, previous):
        return self.filter_alpha * current + (1.0 - self.filter_alpha) * previous

    def rate_limit(self, target, current):
        delta = target - current

        if delta > self.max_speed_change:
            delta = self.max_speed_change
        elif delta < -self.max_speed_change:
            delta = -self.max_speed_change

        return current + delta

    # -------------------------------------------------
    # READ SERIAL
    # -------------------------------------------------
    def joy_callback(self, msg: Joy):
    
        try:
            
            # -----------------------------------------
            # INPUT PARSE
            # -----------------------------------------
            yonelim = (msg.axes[0] - 1500) / 1250.0
            throttle = (msg.axes[1] - 1500) / 750.0
            
            if(msg.buttons[0] == 1):   #STANDBY
                self.mode = 0
            elif(msg.buttons[3] == 0 and msg.buttons[4] == 1 and msg.buttons[5] == 0):   #MANUEL
                self.mode = 1
            elif(msg.buttons[3] == 0 and msg.buttons[4] == 0 and msg.buttons[5] == 0):   #ROBOT KOL
                self.mode = 2
            elif(msg.buttons[3] == 0 and msg.buttons[4] == 0 and msg.buttons[5] == 1):   #OTONOM
                self.mode = 3
             


            
            if((self.get_clock().now() - self.last_otonom_time).nanoseconds * 1e-9 > 0.5):
                self.otonom_linear_x  = 0.0
                self.otonom_linear_y  = 0.0
                self.otonom_angular_x = 0.0
                self.otonom_angular_y = 0.0
                self.otonom_angular_z = 0.0

            # -----------------------------------------
            # BUILD MESSAGE
            # -----------------------------------------
            if self.mode == 0:
                # Standby
                if(msg.buttons[7] == 1):
                    self.get_logger().info(f"Kp inital:{self.kp} Enter Kp:")
                    self.kp = float(input())
                    self.get_logger().info(f"Ki inital:{self.ki} Enter Ki:")
                    self.ki = float(input())
                    self.get_logger().info(f"Kd inital:{self.kd} Enter Kd:")
                    self.kd = float(input())
                self.joy_msg.solhiz = 0.0
                self.joy_msg.saghiz = 0.0
                self.joy_msg.mode = 0

                self.joy_msg.shoulder = 0.0
                self.joy_msg.elbow = 0.0
                self.joy_msg.gripper_right = 0.0
                self.joy_msg.gripper_left = 0.0
                self.joy_msg.base = 0.0
                
                self.joy_msg.kp = self.kp
                self.joy_msg.kd = self.kd
                self.joy_msg.ki = self.ki          
                self.get_logger().info(f"Standby")

            elif self.mode == 1: # MANUEL

            # Normalize joystick
                yonelim_raw  = msg.axes[0]
                yonelim_raw  = yonelim_raw
                throttle_raw = msg.axes[1] 
                throttle_raw = throttle_raw

            # Low-pass filter
                self.yonelim_filtered  = self.low_pass_filter(yonelim_raw,  self.yonelim_filtered)
                self.throttle_filtered = self.low_pass_filter(throttle_raw, self.throttle_filtered)

                yonelim  = self.yonelim_filtered
                throttle = self.throttle_filtered

            # Differential drive mixing
                sol_hiz_target = (throttle - yonelim * 2.0) * 4.0
                sag_hiz_target = (throttle + yonelim * 2.0) * 4.0

            # Clamp
                sol_hiz_target = max(-2.0, min(2.0, sol_hiz_target))
                sag_hiz_target = max(-2.0, min(2.0, sag_hiz_target))

            # Rate limiter
                self.sol_hiz_cmd = self.rate_limit(sol_hiz_target, self.sol_hiz_cmd)
                self.sag_hiz_cmd = self.rate_limit(sag_hiz_target, self.sag_hiz_cmd)
                self.get_logger().info(
                         f"DRIVE | saghiz ={self.joy_msg.saghiz:.3f} "
                         f"solhiz = {self.joy_msg.solhiz:.3f} "
                         )

            # Final deadband
                if abs(self.sol_hiz_cmd) < 0.02:
                    self.sol_hiz_cmd = 0.0
                if abs(self.sag_hiz_cmd) < 0.02:
                    self.sag_hiz_cmd = 0.0
                    
                self.joy_msg.solhiz = self.sol_hiz_cmd
                self.joy_msg.saghiz = self.sag_hiz_cmd
                self.joy_msg.mode = 1
                
                self.joy_msg.shoulder = 0.0
                self.joy_msg.elbow = 0.0
                self.joy_msg.gripper_right = 0.0
                self.joy_msg.gripper_left = 0.0
                self.joy_msg.base = 0.0

            elif self.mode == 2:
                # Arm mode
                
                if(msg.buttons[7] == 1):   # Gripper Mode
                    up_down  = msg.axes[0] 
                    rotate   = msg.axes[1]
                    self.joy_msg.gripper_right = up_down + rotate
                    self.joy_msg.gripper_left = up_down - rotate
                    self.joy_msg.shoulder = 0.0
                    self.joy_msg.elbow = 0.0
                    self.joy_msg.base = 0.0
                    self.get_logger().info(
                         f"GRIPPER | servo={self.joy_msg.servo} "
                         f"gripper_right = {self.joy_msg.gripper_right:.2f} "
                         f"gripper_left = {self.joy_msg.gripper_left:.2f}"
                         )
                
                else:
                    shoulder = msg.axes[1]
                    elbow = msg.axes[0] 

                    if abs(shoulder) < 0.01:
                        shoulder = 0.0

                    if abs(elbow) < 0.01:
                        elbow = 0.0
                    self.joy_msg.shoulder = shoulder
                    self.joy_msg.elbow = elbow
                    self.joy_msg.base  = -msg.axes[3] * 5
                    self.joy_msg.gripper_right = 0.0
                    self.joy_msg.gripper_left = 0.0
                        
                    self.get_logger().info(
                    f"ARM | shoulder={self.joy_msg.shoulder:.2f} "
                    f"elbow={elbow:.2f} "
                    f"base = {self.joy_msg.base:.2f} "
                    f"servo = {self.joy_msg.servo} "
                    )
                         
                
                self.joy_msg.solhiz = 0.0
                self.joy_msg.saghiz = 0.0
                self.joy_msg.mode = 2
                
                    
                self.joy_msg.servo = int(msg.axes[4] * 1000 + 1500)
                

                
                
            elif self.mode == 3: #OTONOM
                self.joy_msg.mode  = 3
                
                self.joy_msg.solhiz = self.clamp_wheel_speed(float((self.otonom_linear_x  ) - (self.otonom_angular_z / 1.5))) 
                self.joy_msg.saghiz = self.clamp_wheel_speed(float((self.otonom_linear_x ) + (self.otonom_angular_z / 1.5)))
                 
                   
                self.get_logger().info(f"OTONOM | L={self.joy_msg.solhiz:.2f} "
                    f"R={self.joy_msg.saghiz:.2f}")
                    
            elif self.mode == 4:  #DRILL
                
                self.joy_msg.solhiz = 0.0
                self.joy_msg.saghiz = 0.0
                self.joy_msg.mode = 0

                self.joy_msg.shoulder = 0.0
                self.joy_msg.elbow = 0.0
                self.joy_msg.gripper_right = 0.0
                self.joy_msg.gripper_left = 0.0
                self.joy_msg.gripper_dir  =  0
                self.joy_msg.base = 0.0
                
                self.joy_msg.spun = msg.axes[0]
                self.joy_msg.drill = msg.axes[1]
                self.joy_msg.plastic = msg.axes[2]

            # Publish ROS message
            self.publisher_.publish(self.joy_msg)
            
            

        except Exception as e:
            self.get_logger().warn(f"Parse error: {e}")


# -------------------------------------------------
# MAIN
# -------------------------------------------------
def main(args=None):
    rclpy.init(args=args)

    node = SerialJoystickPublisher()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
