#!/usr/bin/env python3

# SPDX-FileCopyrightText: 2023 ShinagwaKazemaru
# SPDX-License-Identifier: MIT License

import numpy as np

from .server_assistant import db, queue_to_ros, queue_from_ros
import wili_io.utils._queue.queue_packet as qp


def get_motion_num() -> dict:
    # n := number of motion
    n = db().count_motion()
    return {'motion_num': n}


def select_heatmap() -> dict:
    n = db().count_motion()
    gaussians = db().select_fetch_gaussian()
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
