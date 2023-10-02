#!/usr/bin/env python3

# SPDX-FileCopyrightText: 2023 ShinagwaKazemaru
# SPDX-License-Identifier: MIT License

# for type hints
from queue import Queue
from sqlite3 import Connection, Cursor

class ServerAssistant:
    def __init__(self, conn:Connection, queue_from_ros:Queue, queue_to_ros:Queue):
        self.conn = conn
        self.cur = conn.cursor()
        self.queue_from_ros = queue_from_ros
        self.queue_to_ros = queue_to_ros

assistant:ServerAssistant = None

def init_assistant(conn:Connection, queue_from_ros:Queue, queue_to_ros:Queue):
    global assistant
    assistant = ServerAssistant(conn, queue_from_ros, queue_to_ros)


def db_cursor() -> Cursor:
    global assistant
    return assistant.cur


def queue_to_ros() -> Queue:
    global assistant
    return assistant.queue_to_ros


def queue_from_ros() -> Queue:
    global assistant
    return assistant.queue_from_ros