import json
import cv2
import os
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Bool
from geometry_msgs.msg import Vector3
from sensor_msgs.msg import Image
from rcl_interfaces.msg import ParameterDescriptor, SetParametersResult
from core.msg import Cam
from core.srv import AddCamera
from cv_bridge import CvBridge

# NOTE:
        # This program has 2 config dictionaries. 
            # Standard (self.std_camera_config), and Master (self.master_config).
        # Standard can be changed via GUI parameters. Master cannot.
        # Master is referenced in the process of changing cameras.
        # Master always matches what's currently written in the JSON file.
        # At shutdown, depending on parameters, Standard is written to the JSON file.
        # Neither of these should  be confused with self.active_cameras.
        # self.active cameras contains the ips of all of the currently connected cameras.
        # The config files contain all of the currently configured cameras.

        # The ips are then streamed from to write images to the ./img directory and send camera footage to the shipwreck measurement node.
        
        # Written by Jack Frings '26
        # Uses code from Camera Switcher by James Randall '24

class ImageCaptureNode(Node):

    def __init__(self):
        super().__init__('camera_switcher')

        self.log = self.get_logger() # Quick reference for ROS logging

        # Stores IPs of active cameras
        # Filling unused slots with empty dictionaries
        self.active_cameras = []
        # List of video captures to receive camera input from
        self.camera_feeds = []
        
        # Image count for writing to ./img while recording
        # The count is used by all cameras. It felt simpler to code a single count for all images but you could make 
        # separate counts for each camera feed as well. Just make it a list and iterate over it in receive_cameras()
        self.count = 0

        # Number of unnamed cameras. Used in generating IDs for new cams.
        self.unnamed_cams = 0

        # Define config file path
        self.config_path = "/home/jhsrobo/corews/src/img_capture/cam_config.json"
        self.img_write_path = "/home/jhsrobo/corews/src/img_capture/img"

        # Opens the JSON config file.
        self.check_config_integrity()
        self.open_config() # Creates both Standard and Master Config.
        
        # Bridge to convert received frame to ros2 msgs for shipwreck
        self.bridge = CvBridge()

        # Set up a service for accepting new cameras from find_cameras
        self.camera_adder = self.create_service(AddCamera, "add_ops_camera", self.add_camera_callback)
        self.shipwreck_pub = self.create_publisher(Image, "shipwreck", 10)

        self.yaw = -1
        self.yaw_target = 0
        self.yaw_sub = self.create_subscription(Vector3, 'orientation_sensor', self.yaw_callback, 10)
        self.yaw_target_pub = self.create_publisher(Float32, 'yaw_target', 10)
        self.pilot_ctrl_pub = self.create_publisher(Bool, 'photosphere_enabled', 10)

        # Callback to capture the images from each feed to ./img contingent on self.photosphere and to the shipwreck node contingent on self.shipwreck
        frame_rate = 1.0 / 1000.0 
        self.create_timer(frame_rate, self.receive_cameras)

        # Parameter deciding whether or not to actually capture the images and send it to the shipwreck measure node
        self.shipwreck = False 
        self.declare_parameter("Shipwreck", self.shipwreck)

        # Parameter deciding whether or not to send images to photosphere file system
        self.photosphere = False 
        self.declare_parameter("Photosphere", self.photosphere)
        # Parameter regarding updating the cam_config on shutdown
        self.declare_parameter('save_changes_on_shutdown', False)

        self.create_timer(0.1, self.update_parameters)

    def update_parameters(self):
        self.shipwreck = self.get_parameter("Shipwreck").value 
        self.photosphere = self.get_parameter("Photosphere").value
        self.yaw_target = 0 
        self.count = 0

        self.pilot_ctrl_publish()
        self.yaw_target_publish()

    # Receive camera input, write received frames to ./img, and show frames from a given camera to the display.
    def receive_cameras(self):
        # Variable to keep trackof how many photosphere images there are before photosphere runs
        count = 0
        # Iterate over each camera feed and their corresponding ip at the same time
        for feed, ip in zip(self.camera_feeds, self.active_cameras):
            # Read the frame if possible
            # Find the name of that camera
            cam_index = self.get_master_index(ip)
            cam_name = self.master_config[cam_index]["nickname"]

            if self.photosphere and cam_name != "Bottom" and abs(self.yaw_target-self.yaw) < 0.5:
                frame = self.read_frame(feed)
                if frame is not None:
                    # Write the image to the path corresponding to the camera's name
                    cv2.imwrite(f"{self.img_write_path}/{cam_name}/{self.count}.png", frame)
                    self.count += 1

            # If bottom camera is being recorded, send that bottom camera feed to the shipwreck measuring node
            if cam_name == "Bottom" and self.shipwreck:
                img = self.bridge.cv2_to_imgmsg(frame, encoding="passthrough")
                self.shipwreck_pub.publish(img)

        if self.count - count == 3:
            if self.yaw_target == 360:
                self.yaw_target = 0
                self.photosphere = False
                self.pilot_ctrl_publish()
            else:
                self.yaw_target += 40
                self.yaw_target_publish()
        
    
    # Grab the most recent frame from the camera feed
    def read_frame(self, feed):
        success, frame = feed.read()
        if not success:
            return None
        else: return frame

    def pilot_ctrl_publish(self):
        msg = Bool()
        msg.data = self.photosphere
        self.pilot_ctrl_pub.publish(msg)

    def yaw_callback(self, msg):
        self.yaw = msg.x 

    def yaw_target_publish(self):
        msg = Float32()
        msg.data = float(self.yaw_target)
        self.yaw_target_pub.publish(msg)


    # When the AddCamera service is requested (from find_cameras.py), 
    # add the attached IP to the camera dict.
    def add_camera_callback(self, request, response):

        # Check to see if this IP is pre-registered in the config file
        ip_index = self.get_std_index(request.ip)
        if ip_index is None:
            self.create_config_entry(request.ip)
            ip_index = self.get_std_index(request.ip) # Get new index once it's been added to config
        
        # Add the ip and the actual camera stream of the new camera to the node lists
        self.active_cameras.append(request.ip)
        self.camera_feeds.append(cv2.VideoCapture(f"http://{request.ip}:5000"))

        # Create a directory to store images from the given camera feed
        cam_index = self.get_master_index(request.ip)
        cam_name = self.master_config[cam_index]["nickname"]
        os.makedirs(f"{self.img_write_path}/{cam_name}")

        return response

    # Read the existing camera config into a dictionary
    # JSON files are read into nested dictionaries.
    # Also, keep in mind that although the key values are numbers 1-4, they are strings, not ints.
    def open_config(self):
        with open(self.config_path) as f:
            self.std_camera_config = json.load(f)

        # Save a version of the config for refererence when cameras are changed during runtime
        with open(self.config_path) as f:
            self.master_config = json.load(f)


    # Checks if the config file still exists.
    # If it doesn't, it creates a new one.
    def check_config_integrity(self):
        if os.path.isfile(self.config_path): 
            return True
        else:
            empty_dict = {}
            self.log.info("cam_config.json not found")
            self.log.info("Creating new camera config")
            self.log.info("Edit img_capture/cam_config.json to save your settings")
            with open(self.config_path, "w") as f:
                json.dump(empty_dict, f)


    # Scans the standard config for a certain attribute in inner dictionaries
    # Returns the index of that attribute if found
    def get_std_index(self, value):
        for index in self.std_camera_config:
            for key in self.std_camera_config[index]:
                if self.std_camera_config[index][key] == value:
                    return index
        return None
    

    # Scans the master config for a certain attribute in inner dictionaries
    # Returns the index of that attribute if found
    def get_master_index(self, value):
        for index in self.master_config:
            for key in self.master_config[index]:
                if self.master_config[index][key] == value:
                    return index
        return None


    # These functions search their indices for empty dict keys
    # If none are found, they create a new one.
    def find_available_std_index(self):
        for key in self.std_camera_config:
            if self.std_camera_config[key] == 0:
                return key
        return str(len(self.std_camera_config) + 1)


    def find_available_master_index(self):
        for key in self.master_config:
            if self.master_config[key] == 0:
                return key 
        return str(len(self.master_config) + 1)


    # Creates a new entry in the config files for new cameras
    def create_config_entry(self, ip, gripper="Front", ID="unnamed", nickname = "Unnamed"):
        std_index = self.find_available_std_index() # Assign a new index
        master_index = self.find_available_master_index()
        
        # Give each unnamed cam a unique identifier
        if "unnamed" in ID or "Unnamed" in nickname:
            self.unnamed_cams +=1
        if ID == "unnamed":
            ID += str(self.unnamed_cams)
        if nickname == "Unnamed":
            nickname += str(self.unnamed_cams)
        
        self.std_camera_config[std_index] = { "ip" : ip,
                                      "gripper" : gripper,
                                      "nickname" : nickname,
                                      "ID" : ID }
        self.master_config[master_index] = { "ip" : ip,
                                      "gripper" : gripper,
                                      "nickname" : nickname,
                                      "ID" : ID }

        self.write_to_config()
        self.log.info("")
        self.log.info("Created new camera config entry")


    # Updates the contents of the config file with the contents of self.std_camera_config
    # Setting save_changes high means that it writes the std_camera_config to the JSON
    # This means that changes we made during runtime are reflected in the config file.
    # Otherwise, only new cameras will be added to the config file.
    def write_to_config(self, save_changes=False):
        config_json = json.dumps(self.master_config, indent=2)
        if save_changes: 
            config_json = json.dumps(self.std_camera_config, indent=2)
        
        with open(self.config_path, "w") as f:
            f.write(config_json)

    # Simple getter to run at shutdown
    def get_save_changes(self):
        return self.get_parameter("save_changes_on_shutdown").value
    

    def delete_camera_entries(self, nickname):
        target_index = self.get_std_index(nickname)
        while target_index is not None:
            self.std_camera_config.pop(str(target_index))
            target_index = self.get_std_index(nickname)


def main(args=None):
    os.system("rm -rf /home/jhsrobo/corews/src/img_capture/img/*")

    rclpy.init(args=args)

    img_capture = ImageCaptureNode()

    # Runs the program until shutdown is recieved
    try: rclpy.spin(img_capture)
    except KeyboardInterrupt:
        save_changes = img_capture.get_save_changes()
        img_capture.write_to_config(save_changes)

    # On shutdown, kill node
    img_capture.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
