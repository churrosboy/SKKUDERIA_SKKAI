import cv2
import numpy as np
from nav_msgs.msg import OccupancyGrid
from rclpy.qos import QoSProfile, DurabilityPolicy, HistoryPolicy


class GridFilter:
    """Erosion-based occupancy lookup. ROS2 port: pass the owning rclpy `node`
    so it can subscribe to the map topic and log; or feed maps via map_callback()."""

    def __init__(self, node=None, map_topic=None, debug=False):
        self.node = node
        self.resolution = None      # m/pixel
        self.origin = None          # (x, y)
        self.map_data = None
        self.image = None           # OccupancyGrid -> OpenCV image
        self.eroded_image = None
        self.kernel_size = 3
        self.debug = debug
        self.map_topic = map_topic
        self.map_info = None        # original OccupancyGrid info, reused for the debug grid
        self.map_frame = "map"
        # Latched debug view of the eroded grid for rviz.
        self.eroded_pub = None
        if self.node is not None:
            self.eroded_pub = self.node.create_publisher(
                OccupancyGrid, '/planner/avoidance/eroded_map',
                QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                           history=HistoryPolicy.KEEP_LAST))
        if self.node is not None and self.map_topic:
            self.subscribe_to_map(self.map_topic)
        else:
            self._log("No node/map topic provided; feed maps via map_callback().")

    def _log(self, msg):
        if self.node is not None:
            self.node.get_logger().info(str(msg))
        else:
            print(f"[GridFilter] {msg}")

    def subscribe_to_map(self, map_topic):
        self._log(f"Subscribing to map topic: {map_topic}")
        # The map topic is latched (TRANSIENT_LOCAL); match its QoS or the one-shot map is never received.
        map_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                             history=HistoryPolicy.KEEP_LAST)
        self._sub = self.node.create_subscription(
            OccupancyGrid, map_topic, self.map_callback, map_qos)

    def map_callback(self, msg):
        if self.image is None:
            self.resolution = msg.info.resolution
            self.origin = (msg.info.origin.position.x, msg.info.origin.position.y)
            self.map_info = msg.info
            self.map_frame = msg.header.frame_id or "map"
            width, height = msg.info.width, msg.info.height
            image = np.array(msg.data, dtype=np.int8).reshape((height, width))
            # 255: obstacle, 0: free
            self.image = np.where(image == 100, 0, 255).astype(np.uint8)
            self.update_image()
            self._log("Map image initialized.")

    def set_erosion_kernel_size(self, size):
        self.kernel_size = size
        self.update_image()

    def update_image(self):
        if self.image is None:
            self._log("Map image not initialized.")
            return
        kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (self.kernel_size, self.kernel_size))
        self.eroded_image = cv2.erode(self.image, kernel)
        self.publish_eroded()

    def publish_eroded(self):
        """Publish the eroded grid as a latched OccupancyGrid for rviz debugging.
        Re-published whenever the map arrives or the kernel size changes."""
        if self.eroded_pub is None or self.eroded_image is None or self.map_info is None:
            return
        grid = OccupancyGrid()
        grid.header.frame_id = self.map_frame
        grid.header.stamp = self.node.get_clock().now().to_msg()
        grid.info = self.map_info
        # eroded_image: 255 = free, 0 = wall/inflated -> occupancy 0 / 100
        occ = np.where(self.eroded_image == 255, 0, 100).astype(np.int8)
        grid.data = occ.flatten().tolist()
        self.eroded_pub.publish(grid)
        self._log(f"Published eroded map (kernel {self.kernel_size}) on /planner/avoidance/eroded_map")

    def world_to_pixel(self, x, y):
        px = int((x - self.origin[0]) / self.resolution)
        py = int((y - self.origin[1]) / self.resolution)
        return px, py

    def is_point_inside(self, x, y):
        if self.eroded_image is None:
            return False
        px, py = self.world_to_pixel(x, y)
        if px < 0 or py < 0 or px >= self.eroded_image.shape[1] or py >= self.eroded_image.shape[0]:
            return False
        return self.eroded_image[py, px] == 255
