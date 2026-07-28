import rclpy
from rclpy.node import Node
import time
from sensor_msgs.msg import Joy
from rover_msgs.msg import ControllerMsg
from geometry_msgs.msg import Twist



class SerialJoystickPublisher(Node):

    def __init__(self):
        super().__init__("serial_joystick_publisher")

        # ROS publisher
        self.publisher_ = self.create_publisher(
            ControllerMsg,
            "joystick_cmd",
            10
        )
        
        self.joy_sub = self.create_subscription(
            Joy,
            "joy",
            self.joy_callback,
            20
        )
        
        self.otonom_sub = self.create_subscription( Twist,"cmd_vel",self.otonom_callback, 20)

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
        self.filter_alpha      = 0.5

 # ── Wheel speed smoothing ─────────────────────────────
        self.sag_hiz_cmd      = 0.0
        self.sol_hiz_cmd      = 0.0
        self.max_speed_change = 0.5

   

            
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
            return 0.35
        elif -0.35 < value < 0.0:
            return -0.35
        elif value > 0.6:
            return 0.6
        elif value < -0.6:
            return -0.6
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
    def joy_callback(self, msg:Joy):
    
        try:
            # -----------------------------------------
            # INPUT PARSE
            # -----------------------------------------
            yonelim = msg.axes[0]
            throttle = msg.axes[1] 

            if msg.buttons[0]:
                self.mode = 0  #STANDBY
            elif msg.buttons[4] == 1 and msg.buttons[5] == 0:
                self.mode = 1 #DRİVE
            elif msg.buttons[4] == 0 and msg.buttons[5] == 0:   # ROBOT KOL
                self.mode = 2
            elif msg.buttons[4] == 0 and msg.buttons[5] == 1: #OTONOM
                self.mode = 3 
            
            if((self.get_clock().now() - self.last_otonom_time).nanoseconds * 1e-9 > 0.5):
                self.otonom_linear_x  = 0.0
                self.otonom_linear_y  = 0.0
                self.otonom_angular_x = 0.0
                self.otonom_angular_y = 0.0
                self.otonom_angular_z = 0.0


            # -----------------------------------------
            # ARM VALUES
            # -----------------------------------------
            shoulder = msg.axes[1]
            elbow = msg.axes[0] 

            if abs(shoulder) < 0.01:
                shoulder = 0.0

            if abs(elbow) < 0.01:
                elbow = 0.0

            # Gripper pitch
            if 1400 < msg.axes[3] < 1600:
                gripper_pitch = 0
            else:
                gripper_pitch = int(msg.axes[3] - 1500)

            # Gripper roll
            if 1400 < msg.axes[2] < 1600:
                gripper_roll = 0
            else:
                gripper_roll = int(msg.axes[2] - 1500)

            servo = 0
            base = int(msg.axes[5] * 180)

            # -----------------------------------------
            # BUILD MESSAGE
            # -----------------------------------------
            if self.mode == 0:
                # Standby
                if(msg.buttons[7]):
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
                self.joy_msg.gripper_pitch = 0
                self.joy_msg.gripper_roll = 0
                
                self.joy_msg.kp = self.kp
                self.joy_msg.kd = self.kd
                self.joy_msg.ki = self.ki          
                self.get_logger().info(f"Standby")

            elif self.mode == 1:                        # ── DRIVE ──

            # Deadband on raw input
                if abs(msg.axes[0] - 1500) < 20:
                    msg.axes[0] = 1500
                if abs(msg.axes[1] - 1500) < 20:
                    msg.axes[1] = 1500

            # Normalize joystick
                yonelim_raw  = msg.axes[0]
                yonelim_raw = -yonelim_raw
                throttle_raw = msg.axes[1] 
                throttle_raw = -throttle_raw

            # Low-pass filter
                self.yonelim_filtered  = self.low_pass_filter(yonelim_raw,  self.yonelim_filtered)
                self.throttle_filtered = self.low_pass_filter(throttle_raw, self.throttle_filtered)

                yonelim  = self.yonelim_filtered
                throttle = self.throttle_filtered

            # Differential drive mixing
                sol_hiz_target = throttle + yonelim /1.5
                sag_hiz_target = throttle - yonelim /1.5

            # Clamp
                sol_hiz_target = max(-1.0, min(1.0, sol_hiz_target))
                sag_hiz_target = max(-1.0, min(1.0, sag_hiz_target))

            # Rate limiter
                self.sol_hiz_cmd = self.rate_limit(sol_hiz_target, self.sol_hiz_cmd)
                self.sag_hiz_cmd = self.rate_limit(sag_hiz_target, self.sag_hiz_cmd)

            # Final deadband
                if abs(self.sol_hiz_cmd) < 0.02:
                    self.sol_hiz_cmd = 0.0
                if abs(self.sag_hiz_cmd) < 0.02:
                    self.sag_hiz_cmd = 0.0
                    
                self.joy_msg.solhiz = self.sol_hiz_cmd
                self.joy_msg.saghiz = self.sag_hiz_cmd
                self.joy_msg.mode = 1

                self.joy_msg.shoulder = shoulder 
                self.joy_msg.elbow = elbow 
                self.joy_msg.gripper_pitch = gripper_pitch
                self.joy_msg.gripper_roll = gripper_roll
                self.joy_msg.servo = servo
                self.joy_msg.base = base
                
                self.get_logger().info(
                    f"DRIVE | saghiz={self.joy_msg.saghiz:.2f} "
                    f"solhiz={self.joy_msg.solhiz:.2f}"
                )

            elif self.mode == 2:
                # Arm mode
                self.joy_msg.solhiz = 0.0
                self.joy_msg.saghiz = 0.0
                self.joy_msg.mode = 2

                self.joy_msg.shoulder = shoulder 
                self.joy_msg.elbow = elbow 
                self.joy_msg.gripper_pitch = gripper_pitch
                self.joy_msg.gripper_roll = gripper_roll
                self.joy_msg.servo = servo
                self.joy_msg.base = base

                self.get_logger().info(
                    f"Arm | shoulder={shoulder:.2f} "
                    f"elbow={elbow:.2f}"
                )
                
            elif self.mode == 3: #OTONOM
                self.joy_msg.mode  = 3
                
                self.joy_msg.solhiz = self.clamp_wheel_speed(float((self.otonom_linear_x  ) - (self.otonom_angular_z / 1.5))) 
                self.joy_msg.saghiz = self.clamp_wheel_speed(float((self.otonom_linear_x ) + (self.otonom_angular_z / 1.5)))
                 
                   
                self.get_logger().info(f"OTONOM | L={self.joy_msg.solhiz:.2f} "
                    f"R={self.joy_msg.saghiz:.2f}")

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
