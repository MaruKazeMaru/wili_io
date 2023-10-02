#!/usr/bin/env python3

# SPDX-FileCopyrightText: 2023 ShinagwaKazemaru
# SPDX-License-Identifier: MIT License

# for type hint
from queue import Queue

import sqlite3
from http.server import HTTPServer

from .handler import WiliHandler
from .server_assistant import init_assistant


def http_start(db_path:str, queue_from_ros:Queue, queue_to_ros:Queue()):
    init_assistant(db_path, queue_from_ros,  queue_to_ros)
    httpd = HTTPServer(('localhost', 5000), WiliHandler)
    httpd.serve_forever()