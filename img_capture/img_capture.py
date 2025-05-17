import toml, os, cv2
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

class ImageCaptureNode(Node):

    def __init__(self):
        super().__init__('camera_switcher')

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

        for key in self.config.keys():
            ip = self.config[key]["ip"]
            nickname = self.config[key]["nickname"]
            # Make a camera feed object
            self.camera_feeds[nickname] = cv2.VideoCapture(f"tcp://{ip}:5000")
            # Make sure feed opened correctly
            if not self.camera_feeds[nickname].isOpened():
                self.log.info("Failed to aquire camera: " + nickname)
                exit()
            # Creates a directory to store images from the new camera
            os.makedirs(f"{self.img_write_path}/{nickname}")
      
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

        # Read the cameras as much as possible. This seem unintuitive right? If we don't want to overwhelm the 
        # camera feed, why are we constantly reading? Well, that's just it. The camera freezes if we don't 
        # read frames because it's a TCP stream. So we read as much as possible to not negatively impact
        # the pilot's feed. 
        self.create_timer(0, self.read_cameras)

        self.count = 0

    def read_cameras(self):
        for nickname in self.camera_feeds.keys():
            feed = self.camera_feeds[nickname]
            ret, frame = feed.read()
            if self.photosphere and nickname != "Bottom":
                cv2.imwrite(f"{self.img_write_path}/{nickname}/{self.count}.png", frame)
                self.count += 1
            if self.shipwreck and nickname == "Bottom":
                frame = self.bridge.cv2_to_imgmsg(frame, encoding="passthrough")
                self.shipwreck_pub.publish(frame)

    def update_parameters(self):
        change = self.photosphere != self.get_parameter("Photosphere")
        if change:
            self.photosphere = self.get_parameter("Photosphere").value
            self.yaw_target = 0

        self.shipwreck = self.get_parameter("Shipwreck").value

    def check_config_integrity(self):
        if os.path.isfile(self.config_path): 
            with open(self.config_path) as f: 
                self.config = toml.load(f)
            return True 
        return False 

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
