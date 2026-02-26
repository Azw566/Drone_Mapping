import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point
import time


class FrontierDetector(Node):
    def __init__(self):
        super().__init__('frontier_detector')
        self.declare_parameter('map_topic', '/map')
        self.declare_parameter('output_markers', '/frontiers/markers')
        self.declare_parameter('min_frontier_size', 5)
        self.declare_parameter('publish_rate_hz', 2.0)

        map_topic = self.get_parameter('map_topic').get_parameter_value().string_value
        marker_topic = self.get_parameter('output_markers').get_parameter_value().string_value
        self.min_frontier_size = self.get_parameter('min_frontier_size').get_parameter_value().integer_value

        self.marker_pub = self.create_publisher(MarkerArray, marker_topic, 10)
        self.map_sub = self.create_subscription(OccupancyGrid, map_topic, self.map_cb, 10)

        rate = self.get_parameter('publish_rate_hz').get_parameter_value().double_value
        self.timer = self.create_timer(1.0 / rate, self.publish_markers)

        self.latest_grid = None

    def map_cb(self, msg: OccupancyGrid):
        self.latest_grid = msg

    def publish_markers(self):
        if self.latest_grid is None:
            return
        grid = self.latest_grid
        data = grid.data
        width = grid.info.width
        height = grid.info.height
        res = grid.info.resolution
        origin = grid.info.origin

        def idx(x, y):
            return y * width + x

        frontier_cells = []
        for y in range(height):
            for x in range(width):
                v = data[idx(x, y)]
                if v != 0:  # only free cells (0)
                    continue
                # check 4-neighbors for unknown (-1)
                unknown_neighbor = False
                if x > 0 and data[idx(x - 1, y)] == -1:
                    unknown_neighbor = True
                elif x < width - 1 and data[idx(x + 1, y)] == -1:
                    unknown_neighbor = True
                elif y > 0 and data[idx(x, y - 1)] == -1:
                    unknown_neighbor = True
                elif y < height - 1 and data[idx(x, y + 1)] == -1:
                    unknown_neighbor = True
                if unknown_neighbor:
                    frontier_cells.append((x, y))

        if len(frontier_cells) < self.min_frontier_size:
            return

        markers = MarkerArray()
        m = Marker()
        m.header.frame_id = grid.header.frame_id
        m.header.stamp = self.get_clock().now().to_msg()
        m.ns = 'frontiers'
        m.id = 0
        m.type = Marker.CUBE_LIST
        m.action = Marker.ADD
        m.scale.x = res
        m.scale.y = res
        m.scale.z = 0.05
        m.color.r = 1.0
        m.color.g = 0.5
        m.color.b = 0.0
        m.color.a = 0.8
        m.points = []

        for x, y in frontier_cells:
            pt = Point()
            pt.x = origin.position.x + (x + 0.5) * res
            pt.y = origin.position.y + (y + 0.5) * res
            pt.z = origin.position.z
            m.points.append(pt)

        markers.markers.append(m)
        self.marker_pub.publish(markers)


def main():
    rclpy.init()
    node = FrontierDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
