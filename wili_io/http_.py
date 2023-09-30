#!/usr/bin/env python3

# SPDX-FileCopyrightText: 2023 ShinagwaKazemaru
# SPDX-License-Identifier: MIT License

# for type hints
# from socketserver import _AfInetAddress
from queue import Queue
from sqlite3 import Connection

# from const import REQUEST_TR_PROB
import sqlite3
from http.server import HTTPServer, BaseHTTPRequestHandler
import numpy as np
import json
import urllib.parse
import wili_io.queue_packet as qp

class ServerAssistant:
    def __init__(self, conn:Connection, queue_from_ros:Queue, queue_to_ros:Queue):
        self.conn = conn
        self.cur = conn.cursor()
        self.queue_from_ros = queue_from_ros
        self.queue_to_ros = queue_to_ros


class ServerError(Exception):
    def __init__(self, message:str):
        super().__init__()
        self.message = message


server_assistant:ServerAssistant = None

def motion_num() -> int:
    server_assistant.cur.execute('SELECT COUNT(id) FROM motion')
    return (server_assistant.cur.fetchone())[0]


def get_motion_num() -> dict:
    # n := number of motion
    n = motion_num()
    return {'motion_num': n}


def select_heatmap() -> dict:
    n = motion_num()
    server_assistant.cur.execute('SELECT avr_x, avr_y, var_xx, var_xy, var_yy FROM gaussian ORDER BY motion')
    gaussians = server_assistant.cur.fetchall()
    gaussians = np.array(gaussians, dtype='float32')

    return {'motion_num': n, 'avr': gaussians[:,0:2].tolist(), 'var': gaussians[:,2:5].tolist()}


def call_suggest() -> dict:
    server_assistant.queue_to_ros.put(qp.QueuePacket(header=qp.HEADER_SUGGEST))
    q_res = server_assistant.queue_from_ros.get()
    if q_res.header == qp.HEADER_SUGGEST:
        return {'motion_num': q_res.body.motion_num, 'weight': list(q_res.body.weight)}
    else:
        raise qp.UnexpectedQueuePacket()


ROUTE_VIEW = {
    '/motion_num': get_motion_num,
    '/heatmap': select_heatmap,
    '/suggest': call_suggest,
}


class WiliHandler(BaseHTTPRequestHandler):
    def do_GET(self):
        route = urllib.parse.urlparse(self.path).path
        if not route in ROUTE_VIEW:
            self.send_error(404)
            return

        try:
            body_as_dict = ROUTE_VIEW[route]()
        except sqlite3.ProgrammingError:
            self.send_error(500, message='error in database operation')
            return
        except qp.UnexpectedQueuePacket:
            self.send_error(500, message='error in queue transmission')
            return

        # print('~~~~~')
        # print(body_as_dict)
        # print('~~~~~')

        self.send_response(200)
        self.send_header('Content-Type', 'application/json')
        self.end_headers()
        self.wfile.write(json.dumps(body_as_dict).encode())
        return


def http_start(db_path:str, queue_from_ros:Queue, queue_to_ros:Queue()):
    global server_assistant
    conn = sqlite3.connect(db_path)
    server_assistant = ServerAssistant(conn, queue_from_ros,  queue_to_ros)
    httpd = HTTPServer(('localhost', 5000), WiliHandler)
    httpd.serve_forever()


if __name__ == '__main__':
    import threading
    threading.Thread(target=http_start, args=('/var/lib/wili/test/db.sqlite3',)).start()
