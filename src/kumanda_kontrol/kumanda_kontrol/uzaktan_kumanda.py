import rclpy
from rclpy.node import Node
import serial
import serial.tools.list_ports
import time

from rover_msgs.msg import ControllerMsg
from geometry_msgs.msg import Twist


BAUD_RATE = 115200
SERIAL_PORT = "/dev/ttyUSB0"


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
        
        self.yonelım_filtered = 0.0
        self.throttle_filtered = 0.0
        self.filter_alpha = 0.15
        
        self.sag_hiz_cmd  =0.0
        self.sol_hiz_cmd = 0.0
        self.max_speed_change = 0.04

        # Open serial
        self.connect_serial()

        # Fast timer for low latency
        self.create_timer(0.1, self.read_serial)   

    # -------------------------------------------------
    # SERIAL CONNECTION
    # -------------------------------------------------
    def connect_serial(self):
        try:
            self.ser = serial.Serial(
                SERIAL_PORT,
                BAUD_RATE,
                timeout=0.01
            )

            # Wait for Arduino/STM reset
            time.sleep(2)

            # Clear old buffered data
            self.ser.reset_input_buffer()

            self.get_logger().info(f"Connected to {SERIAL_PORT}")

        except Exception as e:
            self.get_logger().error(f"Serial connection failed: {e}")
            self.ser = None
            
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
        return self.filter_alpha * current + (1.0 -self.filter_alpha) * previous
        
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
    def read_serial(self):

        if self.ser is None:
            self.connect_serial()
            return

        if not self.ser.is_open:
            self.connect_serial()
            return

        latest_line = None

        try:
            # Flush all old lines, keep newest only
            while self.ser.in_waiting > 0:
                latest_line = self.ser.readline() \
                    .decode("utf-8", errors="ignore") \
                    .strip()

        except Exception as e:
            self.get_logger().warn(f"Serial read error: {e}")
            return

        if not latest_line:
            return

        

        try:
            numbers = list(map(int, latest_line.split()))

            # Need 8 values (0..7)
            if len(numbers) < 8:
                self.get_logger().warn("Incomplete data")
                return

            # -----------------------------------------
            # INPUT PARSE
            # -----------------------------------------
            yonelim = (numbers[0] - 1500) / 1250.0
            throttle = (numbers[1] - 1500) / 750.0

            standby = numbers[4]
            vehicle_mode = numbers[8]
            
            if((self.get_clock().now() - self.last_otonom_time).nanoseconds * 1e-9 > 0.5):
                self.otonom_linear_x  = 0.0
                self.otonom_linear_y  = 0.0
                self.otonom_angular_x = 0.0
                self.otonom_angular_y = 0.0
                self.otonom_angular_z = 0.0

            # -----------------------------------------
            # MODE SELECT
            # -----------------------------------------
            if standby == 1000:   #STANDBY
                self.mode = 0

            elif vehicle_mode == 1000: #DRİVE
                self.mode = 1

            elif vehicle_mode == 1500: #ROBOT KOL
                self.mode = 2
            elif vehicle_mode == 2000: #OTONOM
                self.mode = 3

            # -----------------------------------------
            # ARM VALUES
            # -----------------------------------------
            shoulder = (numbers[1] - 1500) / 1000.0
            elbow = (numbers[0] - 1500) / 1000.0

            if abs(shoulder) < 0.01:
                shoulder = 0.0

            if abs(elbow) < 0.01:
                elbow = 0.0

            # Gripper pitch
            if 1400 < numbers[3] < 1600:
                gripper_pitch = 0
            else:
                gripper_pitch = numbers[3] - 1500

            # Gripper roll
            if 1400 < numbers[2] < 1600:
                gripper_roll = 0
            else:
                gripper_roll = numbers[2] - 1500

            servo = numbers[6] + 100
            base = int((numbers[7] - 1000) / 1000.0 * 220) + 180

            # -----------------------------------------
            # BUILD MESSAGE
            # -----------------------------------------
            if self.mode == 0:
                # Standby
                if(numbers[9] == 2000):
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
                self.get_logger().info(f"yusdab")

            elif self.mode == 1:
                # Drive mode
                if(abs(numbers[0] -1500) < 20):
                    numbers[0] = 1500
                    
                if(abs(numbers[1] - 1500) < 20):
                    numbers[1] = 1500
                
                yonelim_raw = (numbers[0] -1500) /1250.0
                throttle_raw = (numbers[1] - 1500)/750.0
                
                self.yonelim_filtered = self.low_pass_filter(yonelim_raw , self.yonelim_filtered)
                self.throttle_filtered = self.low_pass_filter(throttle_raw , self.throttle_filtered)
                
                yonelim = self.yonelim_filtered
                throttle = self.throttle_filtered
                
                sol_hiz_target = (throttle + yonelim*2.0)*2.0
                sag_hiz_target = (throttle - yonelim*2.0)*2.0
                
                sol_hiz_target = max(-1.0 , min(1.0,sol_hiz_target))
                sag_hiz_target = max(-1.0 , min(1.0,sag_hiz_target))
                
                self.sol_hiz_cmd = self.rate_limit(sol_hiz_target, self.sol_hiz_cmd)
                self.sag_hiz_cmd = self.rate_limit(sag_hiz_target, self.sag_hiz_cmd)
                
                if(abs(self.sol_hiz_cmd) < 0.02):
                    self.sol_hiz_cmd = 0.0
                if(abs(self.sag_hiz_cmd) < 0.02):
                    self.sag_hiz_cmd = 0.0
                
                self.joy_msg.solhiz = self.sol_hiz_cmd
                self.joy_msg.saghiz = self.sag_hiz_cmd
                self.joy_msg.mode = 1
                #self.get_logger().info(f"{yonelim} {throttle}")
                self.joy_msg.shoulder = 0.0
                self.joy_msg.elbow = 0.0
                self.joy_msg.gripper_pitch = 0
                self.joy_msg.gripper_roll = 0

                self.get_logger().info(
                   f"Drive | L={self.joy_msg.solhiz:.2f} "
                    f"R={self.joy_msg.saghiz:.2f}"
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
                
                self.joy_msg.solhiz = self.clamp_wheel_speed(float((self.otonom_linear_x  ) - (self.otonom_angular_z / 2.5))) 
                self.joy_msg.saghiz = self.clamp_wheel_speed(float((self.otonom_linear_x ) + (self.otonom_angular_z / 2.5)))
                 
                   
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
