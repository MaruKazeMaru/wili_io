#!/usr/bin/env python3

# SPDX-FileCopyrightText: 2023 ShinagwaKazemaru
# SPDX-License-Identifier: MIT License

import sqlite3
import rclpy
from rclpy.parameter import Parameter
from rclpy.node import Node
from wili_msgs.msg import HMM, Gaussian, Heatmap
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

        hmm = HMM()

        # get number of motion
        self.db_cur.execute('SELECT COUNT(id) FROM motion')
        hmm.motion_num = self.db_cur.fetchone()

        # get transition probabilities
        self.db_cur.execute('SELECT elem FROM tr_prob ORDER BY from_motion, to_motion')
        hmm.tr_prob = [r[0] for r in self.db_cur.fetchall()]

        # get gaussian of each motion
        self.db_cur.execute('SELECT avr_x, avr_y, var_xx, var_xy, var_yy,  FROM gaussian ORDER BY motion')
        gs = []
        for r in self.db_cur.fetchall():
            g = Gaussian()
            g.avr_x = r[0]
            g.avr_y = r[1]
            g.var_xx = r[2]
            g.var_xy = r[3]
            g.var_yy = r[4]
            gs.append(g)

        for i in range(hmm.motion_num):
            heatmap = Heatmap()
            heatmap.gaussian = g[i]
            hmm.heatmaps.append(heatmap)

        res.hmm = hmm
        return res


def main():
    rclpy.init()
    node = DBProxy('/var/lib/wili/test/db.sqlite3')
    node.logger.info('start')
    try:
        rclpy.spin(node)
    except KeyboardInterrupt: print('')
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == "__main__":
    main()