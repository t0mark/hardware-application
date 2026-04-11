import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import paho.mqtt.client as mqtt


class MqttBridgeNode(Node):
    def __init__(self):
        super().__init__('mqtt_bridge')

        self.declare_parameter('host', 'localhost')
        self.declare_parameter('port', 1883)
        self.declare_parameter('mqtt_topic', 'p_zone/cmd')
        self.declare_parameter('ros_topic', '/p_zone/command')

        host = self.get_parameter('host').value
        port = self.get_parameter('port').value
        self.mqtt_topic = self.get_parameter('mqtt_topic').value
        ros_topic = self.get_parameter('ros_topic').value

        self._pub = self.create_publisher(String, ros_topic, 10)

        self._client = mqtt.Client()
        self._client.on_connect = self._on_connect
        self._client.on_message = self._on_message

        self._client.connect(host, port)
        self._client.loop_start()

        self.get_logger().info(
            f'MQTT Bridge started: {host}:{port} [{self.mqtt_topic}] → {ros_topic}'
        )

    def _on_connect(self, client, userdata, flags, rc):
        if rc == 0:
            self.get_logger().info('MQTT connected')
            client.subscribe(self.mqtt_topic)
        else:
            self.get_logger().error(f'MQTT connection failed, rc={rc}')

    def _on_message(self, client, userdata, msg):
        payload = msg.payload.decode('utf-8').strip()
        self.get_logger().info(f'MQTT → ROS: "{payload}"')
        self._pub.publish(String(data=payload))

    def destroy_node(self):
        self._client.loop_stop()
        self._client.disconnect()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = MqttBridgeNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
