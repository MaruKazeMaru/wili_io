import rclpy
from rclpy.node import Node
from rclpy import Future
import socket
from queue import Queue
from threading import Thread
import struct
import os
import numpy as np

from wili_io.utils._sqlite3.db import DBManager
from wili_msgs.srv import Suggest

class SocketBridge(Node):
    def __init__(self, socket_file_path:str):
        super().__init__('socket_bridge')
        self.logger = self.get_logger()

        self.socket_file_path = socket_file_path
        self.socket = socket.socket( \
            family=socket.AF_UNIX, \
            type=socket.SOCK_STREAM, \
            proto=0 \
        )
        self.socket.bind(socket_file_path)
        self.socket.listen()

        self.calls = {
            b'a': self.call_motion_num,
            b'c': self.call_heatmap,
            b'd': self.call_suggest,
        }

        self.db = DBManager('/var/wili/db.sqlite3')

        self.cli_suggest = self.create_client(Suggest, 'suggest')


    def destroy_node(self):
        self.socket.shutdown(socket.SHUT_RDWR)
        if os.path.exists(self.socket_file_path):
            os.remove(self.socket_file_path)
        super().destroy_node()


    def serve(self):
        while True:
            conn, _ = self.socket.accept()
            thread = Thread(target=self.call, args=(conn,))
            thread.setDaemon(True)
            thread.start()


    def call(self, conn:socket.socket):
        req = conn.recv(1024)
        res = self.calls[req]()
        conn.send(res)
        conn.close()


    def call_motion_num(self) -> bytes:
        return self.db.count_motion().to_bytes(1, 'little')


    def call_heatmap(self) -> bytes:
        n = self.db.count_motion()
        gaussians = self.db.select_fetch_gaussian()

        fmt = '<' + ('f' * 5)
        gaussians_bytes = [struct.pack(fmt, *(gaussians[i])) for i in range(n)]

        n_bytes = n.to_bytes(1, 'little')

        return n_bytes + b''.join(gaussians_bytes)


    def call_suggest(self) -> bytes:
        future:Future = self.cli_suggest.call_async(Suggest.Request())
        while rclpy.ok() and (not future.done()):
            rclpy.spin_once(self)
        res:Suggest.Response = future.result()
        fmt = 'f' * res.motion_num
        n_bytes = res.motion_num.to_bytes(1, 'little')
        w_bytes = struct.pack('<' + fmt, *(res.weight))
        return n_bytes + w_bytes


def main():
    socket_sile_path = '/tmp/wili/socket.socket'

    rclpy.init()
    node = SocketBridge(socket_sile_path)
    node.logger.info('start')
    try:
        node.serve()
    except KeyboardInterrupt: print('')
    node.destroy_node()
    rclpy.try_shutdown()
