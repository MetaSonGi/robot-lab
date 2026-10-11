"""Optional ROS 2 JointState bridge. Requires a sourced ROS 2 environment.
This connects to the lab only, never to hardware. Not exercised in the build environment.
"""
import asyncio
import json
import queue
import threading
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import websockets

class Bridge(Node):
    def __init__(self):
        super().__init__('hyoki_robot_lab_bridge')
        self.declare_parameter('robot','go2')
        self.declare_parameter('port',8000)
        self.commands=queue.Queue(maxsize=100)
        self.latest=None
        self.names={}
        self.lock=threading.Lock()
        self.publisher=self.create_publisher(JointState,'/robot_lab/joint_states',10)
        self.create_subscription(JointState,'/robot_lab/joint_targets',self.target,10)
        self.create_timer(1/30,self.publish_state)
        self.thread=threading.Thread(target=self.network,daemon=True)
        self.thread.start()

    def target(self,msg):
        if len(msg.name)!=len(msg.position):
            self.get_logger().error('name과 position 길이가 달라요');return
        for name,q in zip(msg.name,msg.position):
            with self.lock:j=self.names.get(name)
            if j is not None:
                try:self.commands.put_nowait({'type':'joint','joint':j,'target':float(q)})
                except queue.Full:self.get_logger().warning('명령 큐 가득 참')

    def publish_state(self):
        with self.lock:
            state=self.latest;names=dict(self.names)
        if not state:return
        reverse={j:n for n,j in names.items()}
        msg=JointState();msg.header.stamp=self.get_clock().now().to_msg()
        msg.name=[reverse[j['id']] for j in state['joints']]
        msg.position=[j['q'] for j in state['joints']]
        msg.velocity=[j['dq'] for j in state['joints']]
        msg.effort=[j['torque'] for j in state['joints']]
        self.publisher.publish(msg)

    def network(self):
        try:asyncio.run(self.run_socket())
        except Exception as e:self.get_logger().error(f'통신 종료: {e}')

    async def run_socket(self):
        robot=self.get_parameter('robot').value
        port=int(self.get_parameter('port').value)
        async with websockets.connect(f'ws://127.0.0.1:{port}/ws',max_size=64*1024*1024) as ws:
            await ws.send(json.dumps({'type':'select','robot':robot}))
            started=False
            while rclpy.ok():
                try:
                    msg=json.loads(await asyncio.wait_for(ws.recv(),timeout=.1))
                    if msg['type']=='model' and msg['robot']==robot:
                        with self.lock:self.names={j['name']:j['id'] for j in msg['joints']};self.latest=None
                        await ws.send(json.dumps({'type':'play'}));started=True
                    elif msg['type']=='state' and started:
                        with self.lock:self.latest=msg
                    elif msg['type']=='error':self.get_logger().error(msg['message'])
                except asyncio.TimeoutError:pass
                if started:
                    while not self.commands.empty():
                        await ws.send(json.dumps(self.commands.get_nowait()))

if __name__=='__main__':
    rclpy.init();node=Bridge()
    try:rclpy.spin(node)
    except KeyboardInterrupt:pass
    finally:node.destroy_node();rclpy.shutdown()
