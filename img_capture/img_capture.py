import toml, os, cv2
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Vector3
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

class ImageCaptureNode(Node):

    def __init__(self):
        super().__init__('img_capture')

        self.log = self.get_logger() # Quick reference for ROS logging

        # Opens the preloaded cam_config.toml file. We could've just ran a flask server like in the pilot_gui
        # program to automatically detect cameras but we'd have to name them anyway in the config and I 
        # don't want to store uneccesary cameras in the config.
        self.config_path = "/home/jhsrobo/corews/src/img_capture/cam_config.toml"
        self.img_write_path = "/home/jhsrobo/corews/src/img_capture/img"

        # If config exists, read from the toml file. Else, quit the program.
        if not self.check_config_integrity():
            self.log.info("User needs to create the camera config")
            exit()

        # Bridge to convert received frame to ros2 msgs for shipwreck
        self.bridge = CvBridge()
       
        # Dictionary to store cv2 objects for each camera stream
        # The key for each value is the camera nickname. The value is the cv2 object.
        self.camera_feeds = {}
        self.log.info(str(self.config))
        for key in self.config.keys():
            ip = self.config[key]["ip"]
            nickname = self.config[key]["nickname"]
            # Make a camera feed object
            self.camera_feeds[nickname] = cv2.VideoCapture(f"http://{ip}:5000")
            # Make sure feed opened correctly
            if not self.camera_feeds[nickname].isOpened():
                self.log.info("Failed to aquire camera: " + nickname)
                exit()
            # Creates a directory to store images from the new camera
            os.makedirs(f"{self.img_write_path}/{nickname}")
     
        # We use the pilot_ctrl publisher to spin the ROV in a circle while taking photos before return control
        # to the pilot.
        self.yaw = 0
        self.yaw_taget = -1
        self.yaw_sub = self.create_subscription(Vector3, "orientation_sensor", self.yaw_callback, 10)
        self.pilot_ctrl_pub = self.create_publisher(Bool, "photosphere_enabled", 10)

        # Publisher to send the camera feed to the shipwreck node
        self.shipwreck_pub = self.create_publisher(Image, "shipwreck", 10)

        # Parameter to determine whether or not to receive the camera stream from the MEH, Photosphere-U, 
        # and Photosphere-D cameras.
        self.photosphere = False 
        self.declare_parameter("Photosphere", self.photosphere)
        
        # Parameter to determine whether or not to send the bottom camera feed to the shipwreck measure node
        self.shipwreck = False 
        self.declare_parameter("Shipwreck", self.shipwreck)

        self.create_timer(0.1, self.update_parameters)

        # Reads frames as much as possible so as to not freeze the pilot TCP stream
        self.create_timer(0, self.read_cameras)


    def update_parameters(self):
        change = self.photosphere != self.get_parameter("Photosphere")
        if change:
            self.photosphere = self.get_parameter("Photosphere").value
            self.yaw_target = 0
            self.pilot_ctrl_publish()

        self.shipwreck = self.get_parameter("Shipwreck").value

    def pilot_ctrl_publish(self):
        msg = Bool()
        msg.data = self.photosphere 
        self.pilot_ctrl_pub.publish(msg)

    def yaw_callback(self, msg):
        self.yaw = msg.x

    # Returns true if the cam_config.toml file exists. Otherwise, return false.
    def check_config_integrity(self):
        if os.path.isfile(self.config_path): 
            with open(self.config_path) as f: 
                self.config = toml.load(f)
            return True 
        return False 

    def read_cameras(self):
        change_target = False 
        for nickname in self.camera_feeds.keys():
            feed = self.camera_feeds[nickname]
            ret, frame = feed.read()
            if self.photosphere and nickname != "Bottom" and (abs(self.yaw_target-self.yaw) < 0.5 or abs(self.yaw_target-self.yaw) > 355.5):
                cv2.imwrite(f"{self.img_write_path}/{nickname}/{self.yaw_target}.jpg", frame, [int(cv2.IMWRITE_JPEG_QUALITY), 95])
                change_target = True
            if self.shipwreck and nickname == "Bottom":
                frame = self.bridge.cv2_to_imgmsg(frame, encoding="passthrough")
                self.shipwreck_pub.publish(frame)

        if change_target:
            if self.yaw_target == 360:
                self.yaw_target = -1 
                self.yaw = 0 
                self.photosphere = False 
                self.pilot_ctrl_publish()
            else:
                self.yaw_target += 90


def main(args=None):
    os.system("rm -rf /home/jhsrobo/corews/src/img_capture/img/*")

    rclpy.init(args=args)

    img_capture = ImageCaptureNode()

    # Runs the program until shutdown is recieved
    rclpy.spin(img_capture)

    # On shutdown, kill node
    img_capture.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
