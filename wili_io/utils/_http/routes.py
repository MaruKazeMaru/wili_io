#!/usr/bin/env python3

# SPDX-FileCopyrightText: 2023 ShinagwaKazemaru
# SPDX-License-Identifier: MIT License

import numpy as np
from .server_assistant import db_cursor, queue_to_ros, queue_from_ros
import wili_io.utils._queue.queue_packet as qp


def motion_num() -> int:
    db_cursor().execute('SELECT COUNT(id) FROM motion')
    return (db_cursor().fetchone())[0]


def get_motion_num() -> dict:
    # n := number of motion
    n = motion_num()
    return {'motion_num': n}


def select_heatmap() -> dict:
    n = motion_num()
    db_cursor().execute('SELECT avr_x, avr_y, var_xx, var_xy, var_yy FROM gaussian ORDER BY motion')
    gaussians = db_cursor().fetchall()
    gaussians = np.array(gaussians, dtype='float32')

    return {'motion_num': n, 'avr': gaussians[:,0:2].tolist(), 'var': gaussians[:,2:5].tolist()}


def call_suggest() -> dict:
    queue_to_ros().put(qp.QueuePacket(header=qp.HEADER_SUGGEST))
    q_res = queue_from_ros().get()
    if q_res.header == qp.HEADER_SUGGEST:
        return {'motion_num': q_res.body.motion_num, 'weight': list(q_res.body.weight)}
    else:
        raise qp.UnexpectedQueuePacket()


ROUTES = {
    '/motion_num': get_motion_num,
    '/heatmap': select_heatmap,
    '/suggest': call_suggest,
}
