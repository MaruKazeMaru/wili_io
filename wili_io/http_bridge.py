#!/usr/bin/env python3

# SPDX-FileCopyrightText: 2023 ShinagwaKazemaru
# SPDX-License-Identifier: MIT License

import rclpy
from rclpy.node import Node
from rclpy import Future
from rclpy.client import Client
from queue import Queue, Empty
from threading import Thread
from wili_io.http_ import http_start
import wili_io.queue_packet as qp
from wili_msgs.srv import Suggest

class ServiceBundle:
    #def __init__(self, client:Client, service_result_to_queue_response, future:Future | None = None):
    def __init__(self, client:Client, service_result_to_queue_response):
        self.client = client
        self.service_result_to_queue_response = service_result_to_queue_response
        # self.future = future


class HTTPBridge(Node):
    def __init__(self):
        super().__init__('http_bridge')

        self.logger = self.get_logger()

        self.cli_suggest = self.create_client(Suggest, 'suggest')

        self.on_queue_request = {
            qp.HEADER_SUGGEST: self.call_suggest,
        }


    def call_suggest(self, queue_request_body) -> qp.QueuePacket:
        future = self.cli_suggest.call_async(Suggest.Request())
        rclpy.spin_until_future_complete(self, future)
        return qp.QueuePacket(header=qp.HEADER_SUGGEST, body=future.result())


def main():
    q_from_ros = Queue()
    q_to_ros = Queue()
    Thread(target=http_start, args=('/var/lib/wili/test/db.sqlite3', q_from_ros, q_to_ros)).start()
    rclpy.init()
    node = HTTPBridge()
    node.logger.info('start')
    try:
        while rclpy.ok():
            q_req = q_to_ros.get()
            if q_req.header in node.on_queue_request:
                q_res = node.on_queue_request[q_req.header](q_req.body)
                q_from_ros.put(q_res)
            else:
                q_from_ros.put(qp.QueuePacket(header=qp.HEADER_UNKNOWN))
    except KeyboardInterrupt: print('')
    node.destroy_node()
    rclpy.try_shutdown()
