#!/usr/bin/env python3

# SPDX-FileCopyrightText: 2023 ShinagwaKazemaru
# SPDX-License-Identifier: MIT License

# for type hints
from queue import Queue
from sqlite3 import Connection, Cursor

from .._sqlite3.db import DBManager

class ServerAssistant:
    def __init__(self, db_path:str, queue_from_ros:Queue, queue_to_ros:Queue):
        self.db = DBManager(db_path)
        self.queue_from_ros = queue_from_ros
        self.queue_to_ros = queue_to_ros

assistant:ServerAssistant = None

def init_assistant(db_path:str, queue_from_ros:Queue, queue_to_ros:Queue):
    global assistant
    assistant = ServerAssistant(db_path, queue_from_ros, queue_to_ros)


def db() -> DBManager:
    global assistant
    return assistant.db


def queue_to_ros() -> Queue:
    global assistant
    return assistant.queue_to_ros


def queue_from_ros() -> Queue:
    global assistant
    return assistant.queue_from_ros