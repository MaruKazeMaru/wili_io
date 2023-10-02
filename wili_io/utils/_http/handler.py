#!/usr/bin/env python3

# SPDX-FileCopyrightText: 2023 ShinagwaKazemaru
# SPDX-License-Identifier: MIT License

from http.server import BaseHTTPRequestHandler
import urllib.parse
import sqlite3
import json

from .routes import ROUTES
import wili_io.utils._queue.queue_packet as qp


class WiliHandler(BaseHTTPRequestHandler):
    def do_GET(self):
        route = urllib.parse.urlparse(self.path).path
        if not route in ROUTES:
            self.send_error(404)
            return

        try:
            body_as_dict = ROUTES[route]()
        except (sqlite3.ProgrammingError, sqlite3.OperationalError):
            self.send_error(500, message='error in database operation')
            return
        except qp.UnexpectedQueuePacket:
            self.send_error(500, message='error in queue transmission')
            return

        self.send_response(200)
        self.send_header('Content-Type', 'application/json')
        self.end_headers()
        self.wfile.write(json.dumps(body_as_dict).encode())
        return