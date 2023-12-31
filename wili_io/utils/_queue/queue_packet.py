#!/usr/bin/env python3

# SPDX-FileCopyrightText: 2023 ShinagwaKazemaru
# SPDX-License-Identifier: MIT License

HEADER_SUGGEST = b's'
HEADER_TR_PROB = b't'
HEADER_HEATMAP = b'h'

HEADER_UNKNOWN = b'u'

class QueuePacket:
    def __init__(self, header:bytes, body=None):
        self.header = header
        self.body = body

class UnexpectedQueuePacket(BaseException): pass
