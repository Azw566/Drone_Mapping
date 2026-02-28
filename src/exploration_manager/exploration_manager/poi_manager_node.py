"""
poi_manager_node.py  (singleton)

Central Point-of-Interest registry for all ArUco tag detections.

Subscribes to each drone's detection topic, deduplicates by tag_id
(keeping the highest-confidence sighting), and implements the
RegisterPOI service so that external nodes can also submit detections.

Subscribed
  /{ns}/aruco/detections    drone_interfaces/ArucoDetection   (per drone)

Published
  /poi/all_tags             drone_interfaces/ArucoDetection[] not standard —
                            published as a repeated topic for each known tag

Service server
  /register_poi             drone_interfaces/RegisterPOI
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker, MarkerArray

from drone_interfaces.msg import ArucoDetection
from drone_interfaces.srv import RegisterPOI


class POIManagerNode(Node):
    def __init__(self):
        super().__init__('poi_manager')

        self.declare_parameter('drone_namespaces', ['d1', 'd2'])

        namespaces = (self.get_parameter('drone_namespaces')
                      .get_parameter_value().string_array_value)

        # tag_id → best ArucoDetection seen so far
        self._tags: dict[int, ArucoDetection] = {}

        for ns in namespaces:
            self.create_subscription(
                ArucoDetection, f'/{ns}/aruco/detections',
                lambda msg, n=ns: self._detection_cb(msg), 10)

        self._srv = self.create_service(
            RegisterPOI, '/register_poi', self._register_cb)

        # Re-publish all known tags at 1 Hz for visualisation/debugging
        self.create_timer(1.0, self._publish_all)

        # Publishers
        self._pub         = self.create_publisher(ArucoDetection, '/poi/detections', 10)
        self._pub_markers = self.create_publisher(MarkerArray, '/poi/markers', 10)

        self.get_logger().info(
            f'POIManager watching namespaces: {list(namespaces)}')

    # ── Helpers ────────────────────────────────────────────────────────────
    def _register(self, msg: ArucoDetection) -> tuple[bool, int]:
        """Insert or update; returns (is_new, total_unique_tags)."""
        tid = msg.tag_id
        is_new = tid not in self._tags
        if is_new or msg.confidence > self._tags[tid].confidence:
            self._tags[tid] = msg
            if is_new:
                self.get_logger().info(
                    f'[POI] New tag {tid} detected by {msg.detected_by} '
                    f'(total={len(self._tags)})')
            else:
                self.get_logger().debug(
                    f'[POI] Updated tag {tid} — confidence '
                    f'{self._tags[tid].confidence:.2f} → {msg.confidence:.2f}')
        return is_new, len(self._tags)

    # ── Callbacks ──────────────────────────────────────────────────────────
    def _detection_cb(self, msg: ArucoDetection):
        self._register(msg)

    def _register_cb(self, req: RegisterPOI.Request,
                     res: RegisterPOI.Response) -> RegisterPOI.Response:
        det = ArucoDetection()
        det.tag_id      = req.tag_id
        det.world_pose  = req.world_pose
        det.confidence  = req.confidence
        det.detected_by = req.detected_by
        is_new, total   = self._register(det)
        res.is_new            = is_new
        res.total_tags_found  = total
        return res

    def _publish_all(self):
        ma = MarkerArray()
        for i, det in enumerate(self._tags.values()):
            self._pub.publish(det)

            # Sphere at tag position
            m = Marker()
            m.header          = det.world_pose.header
            m.ns              = 'poi'
            m.id              = int(det.tag_id)
            m.type            = Marker.SPHERE
            m.action          = Marker.ADD
            m.pose            = det.world_pose.pose
            m.scale.x = m.scale.y = m.scale.z = 0.3
            m.color.r = 1.0
            m.color.g = 0.2
            m.color.b = 0.2
            m.color.a = 1.0
            ma.markers.append(m)

            # Text label above the sphere
            t = Marker()
            t.header          = det.world_pose.header
            t.ns              = 'poi_label'
            t.id              = int(det.tag_id)
            t.type            = Marker.TEXT_VIEW_FACING
            t.action          = Marker.ADD
            t.pose            = det.world_pose.pose
            t.pose.position.z += 0.4
            t.scale.z         = 0.25
            t.color.r = t.color.g = t.color.b = 1.0
            t.color.a = 1.0
            t.text            = f'Tag {det.tag_id}\n({det.detected_by})'
            ma.markers.append(t)

        self._pub_markers.publish(ma)


def main():
    rclpy.init()
    node = POIManagerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
