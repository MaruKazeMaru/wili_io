#!/usr/bin/env python3

# SPDX-FileCopyrightText: 2023 ShinagwaKazemaru
# SPDX-License-Identifier: MIT License

import sqlite3
import rclpy
from rclpy.parameter import Parameter
from rclpy.node import Node
from wili_msgs.msg import HMM
from wili_msgs.srv import GetHMM
from math import sqrt

class DBProxy(Node):
    def __init__(self, db_path:str):
        super().__init__("db_proxy")

        self.db_conn = sqlite3.connect(db_path)
        self.db_cur = self.db_conn.cursor()
        self.srv_tr_mat = self.create_service(GetHMM, "get_hmm", self.get_hmm)

        self.logger = self.get_logger()


    def destroy_node(self):
        self.db_conn.close()
        super().destroy_node()


    def get_hmm(self, req:GetHMM.Request, res:GetHMM.Response):
        self.logger.info('service "/get_hmm" called')
        self.db_cur.execute('SELECT elem FROM tr_prob ORDER BY from_motion, to_motion')
        hmm = HMM()
        hmm.tr_prob = [f[0] for f in self.db_cur.fetchall()]
        hmm.motion_num = int(sqrt(len(hmm.tr_prob)))
        res.hmm = hmm
        return res


def main():
    rclpy.init()
    node = DBProxy('../test.db')
    node.logger.info('start')
    try:
        rclpy.spin(node)
    except KeyboardInterrupt: print('')
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == "__main__":
    main()