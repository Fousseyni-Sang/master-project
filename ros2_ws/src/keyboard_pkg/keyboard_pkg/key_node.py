import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, Bool
from pynput import keyboard
import threading

class KeyboardController:
    def __init__(self):
        self.down_pos = 0.
        self.up_pos = 0.
        self.right_pos = 0.
        self.left_pos = 0.
        self.rot_y_plus = 0.
        self.rot_y_minus = 0.
        self.do_translation = True # default behavior when no mode is choosen at the launching
        self.do_rotation = False
        self.detla = 0.15
        self.position_dict = {
            "trans_x": 0., "trans_y": 0., "trans_z": 0., "rot_x": 0., "rot_y": 0., "rot_z": 0.
        }  # contains values of the free axis rotation and translation


    def on_press(self, key):
        # Execute when a key is pressed
        try:
            # choose mode: t for translation and r for rotation
            if key.char == 't':
                self.do_translation = True
                self.do_rotation = False
                print(f"Translation Mode: {self.do_translation}, Rotation Mode: {self.do_rotation}")
            elif key.char == 'r':
                self.do_rotation = True
                self.do_translation = False
                print(f"Translation Mode: {self.do_translation}, Rotation Mode: {self.do_rotation}")
        except AttributeError:
            if key == keyboard.Key.down:
                self.down_pos -= self.detla
                self.down_pos = max(-1., self.down_pos) # to limit value to float -1.
                self.position_dict["trans_y"] = self.down_pos if self.do_translation else 0.
                self.position_dict["rot_x"] = self.down_pos if self.do_rotation else 0.
            elif key == keyboard.Key.up:
                self.up_pos += self.detla
                self.up_pos = min(self.up_pos, 1.)
                self.position_dict["trans_y"] = self.up_pos if self.do_translation else 0.
                self.position_dict["rot_x"] = self.up_pos if self.do_rotation else 0.
            elif key == keyboard.Key.left:
                self.left_pos -= self.detla
                self.left_pos = max(self.left_pos, -1.)
                self.position_dict["trans_x"] = self.left_pos if self.do_translation else 0.
                self.position_dict["rot_z"] = self.left_pos if self.do_rotation else 0.
            elif key == keyboard.Key.right:
                self.right_pos += self.detla
                self.right_pos = min(self.right_pos, 1.)
                self.position_dict["trans_x"] = self.right_pos if self.do_translation else 0.
                self.position_dict["rot_z"] = self.right_pos if self.do_rotation else 0.
            elif key == keyboard.Key.page_down:
                self.rot_y_plus += self.detla
                self.rot_y_plus = min(self.rot_y_plus, 1.)
                self.position_dict["rot_y"] = self.rot_y_plus
            elif key == keyboard.Key.page_up:
                self.rot_y_minus -= self.detla
                self.rot_y_minus = max(self.rot_y_minus, -1.)
                self.position_dict["rot_y"] = self.rot_y_minus
            else:
                print(f"No event for this key: {key}")

            print(self.position_dict)
            

    def on_release(self, key):
        # reset all values to zero when key is released
        self.down_pos = 0.
        self.up_pos = 0.
        self.right_pos = 0.
        self.left_pos = 0.
        self.rot_y_plus = 0.
        self.rot_y_minus = 0.
        self.position_dict = {
            "trans_x": 0., "trans_y": 0., "trans_z": 0., "rot_x": 0., "rot_y": 0., "rot_z": 0.
        }
        if key == keyboard.Key.esc:
            # Stop listener
            return False

    def start_listener(self):
        # listen to keyboard event with non-blocking mode
        listener = keyboard.Listener(
        on_press=self.on_press,
        on_release=self.on_release)
        listener.start()
        


class CustomTopicPublisher(Node):
    def __init__(self):
        super().__init__('key_node')
        self.publisher_ = self.create_publisher(Float32MultiArray, 'key', 10)

        self.keyboard_controller = KeyboardController()
        self.keyboard_controller.start_listener()

        timer_period = 0.001  # seconds
        self.timer = self.create_timer(timer_period, self.publish_data)

    def publish_data(self):
        # publish the data on the topic
        pos_array = self.keyboard_controller.position_dict
        msg = Float32MultiArray()
        
        msg.data = [
            float(pos_array["trans_x"]), float(pos_array["trans_y"]), float(pos_array["trans_z"]),
            float(pos_array["rot_x"]), float(pos_array["rot_y"]), float(pos_array["rot_z"])
        ]
        self.publisher_.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = CustomTopicPublisher()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
