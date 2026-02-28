import collections
import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point
from std_msgs.msg import Header
from drone_interfaces.msg import FrontierList


class FrontierDetector(Node):
    def __init__(self):
        super().__init__('frontier_detector')
        self.declare_parameter('map_topic', '/map')
        self.declare_parameter('output_markers', '/frontiers/markers')
        self.declare_parameter('output_frontiers', '/frontiers/list')
        self.declare_parameter('min_frontier_size', 5)
        self.declare_parameter('publish_rate_hz', 2.0)

        map_topic      = self.get_parameter('map_topic').get_parameter_value().string_value
        marker_topic   = self.get_parameter('output_markers').get_parameter_value().string_value
        frontier_topic = self.get_parameter('output_frontiers').get_parameter_value().string_value
        self.min_frontier_size = self.get_parameter('min_frontier_size').get_parameter_value().integer_value

        self.marker_pub   = self.create_publisher(MarkerArray, marker_topic, 10)
        self.frontier_pub = self.create_publisher(FrontierList, frontier_topic, 10)
        self.map_sub      = self.create_subscription(OccupancyGrid, map_topic, self.map_cb, 10)

        rate = self.get_parameter('publish_rate_hz').get_parameter_value().double_value
        self.timer = self.create_timer(1.0 / rate, self.publish_frontiers)

        self.latest_grid = None

    def map_cb(self, msg: OccupancyGrid):
        self.latest_grid = msg

    def publish_frontiers(self):
        if self.latest_grid is None:
            return

        grid   = self.latest_grid
        data   = grid.data
        width  = grid.info.width
        height = grid.info.height
        res    = grid.info.resolution
        origin = grid.info.origin

        # ── Step 1: collect individual frontier cells ─────────────────────
        def idx(x, y):
            return y * width + x

        is_frontier = {}
        for y in range(height):
            for x in range(width):
                if data[idx(x, y)] != 0:
                    continue
                for dx, dy in ((1, 0), (-1, 0), (0, 1), (0, -1)):
                    nx, ny = x + dx, y + dy
                    if 0 <= nx < width and 0 <= ny < height:
                        if data[idx(nx, ny)] == -1:
                            is_frontier[(x, y)] = True
                            break

        if not is_frontier:
            return

        # ── Step 2: BFS cluster frontier cells into connected components ──
        visited  = set()
        clusters = []

        for seed in is_frontier:
            if seed in visited:
                continue
            cluster = []
            queue   = collections.deque([seed])
            visited.add(seed)
            while queue:
                cx, cy = queue.popleft()
                cluster.append((cx, cy))
                for dx, dy in ((1, 0), (-1, 0), (0, 1), (0, -1)):
                    nb = (cx + dx, cy + dy)
                    if nb in is_frontier and nb not in visited:
                        visited.add(nb)
                        queue.append(nb)
            if len(cluster) >= self.min_frontier_size:
                clusters.append(cluster)

        if not clusters:
            return

        # ── Step 3: compute cluster centroids in world frame ──────────────
        def cell_to_world(x, y):
            wx = origin.position.x + (x + 0.5) * res
            wy = origin.position.y + (y + 0.5) * res
            wz = origin.position.z
            return wx, wy, wz

        stamp    = self.get_clock().now().to_msg()
        frame_id = grid.header.frame_id

        # ── Step 4: publish FrontierList ──────────────────────────────────
        fl = FrontierList()
        fl.header = Header(stamp=stamp, frame_id=frame_id)
        for cluster in clusters:
            avg_x = sum(c[0] for c in cluster) / len(cluster)
            avg_y = sum(c[1] for c in cluster) / len(cluster)
            wx, wy, wz = cell_to_world(avg_x, avg_y)
            pt = Point(x=wx, y=wy, z=wz)
            fl.centroids.append(pt)
            fl.sizes.append(float(len(cluster)))
        self.frontier_pub.publish(fl)

        # ── Step 5: publish MarkerArray (all cells coloured by cluster) ───
        markers = MarkerArray()

        # Delete old markers first
        del_m = Marker()
        del_m.header.frame_id = frame_id
        del_m.header.stamp    = stamp
        del_m.ns              = 'frontiers'
        del_m.id              = 0
        del_m.action          = Marker.DELETEALL
        markers.markers.append(del_m)

        # One CUBE_LIST marker per cluster (coloured differently)
        palette = [
            (1.0, 0.5, 0.0),  # orange
            (0.0, 1.0, 0.5),  # green-cyan
            (0.3, 0.6, 1.0),  # blue
            (1.0, 0.2, 0.8),  # magenta
            (1.0, 1.0, 0.0),  # yellow
        ]
        for i, cluster in enumerate(clusters):
            r, g, b = palette[i % len(palette)]
            m = Marker()
            m.header.frame_id = frame_id
            m.header.stamp    = stamp
            m.ns              = 'frontiers'
            m.id              = i + 1
            m.type            = Marker.CUBE_LIST
            m.action          = Marker.ADD
            m.scale.x = res
            m.scale.y = res
            m.scale.z = 0.05
            m.color.r = r
            m.color.g = g
            m.color.b = b
            m.color.a = 0.8
            for cx, cy in cluster:
                wx, wy, wz = cell_to_world(cx, cy)
                m.points.append(Point(x=wx, y=wy, z=wz))
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
