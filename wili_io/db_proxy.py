#!/usr/bin/env python3

# SPDX-FileCopyrightText: 2023 ShinagwaKazemaru
# SPDX-License-Identifier: MIT License

import rclpy
from rclpy.node import Node

from wili_msgs.msg import HMM, Gaussian, Heatmap
from wili_msgs.srv import GetHMM
from wili_io.utils._sqlite3.db import DBManager

class DBProxy(Node):
    def __init__(self, db_path:str):
        super().__init__("db_proxy")

        self.db = DBManager(db_path)
        self.srv_tr_mat = self.create_service(GetHMM, "get_hmm", self.get_hmm)

        self.logger = self.get_logger()


    def destroy_node(self):
        self.db.conn.close()
        super().destroy_node()


    def get_hmm(self, req:GetHMM.Request, res:GetHMM.Response):
        self.logger.info('service "/get_hmm" called')

        hmm = HMM()

        # get number of motion
        hmm.motion_num = self.db.count_motion()

        # get transition probabilities
        hmm.tr_prob = self.db.select_fetch_tr_prob()

        # get gaussian of each motion
        gs = []
        for r in self.db.select_fetch_gaussian():
            g = Gaussian()
            g.avr_x = r[0]
            g.avr_y = r[1]
            g.var_xx = r[2]
            g.var_xy = r[3]
            g.var_yy = r[4]
            gs.append(g)

        for i in range(hmm.motion_num):
            heatmap = Heatmap()
            heatmap.gaussian = gs[i]
            hmm.heatmaps.append(heatmap)

        res.hmm = hmm
        return res


def main():
    rclpy.init()
    node = DBProxy('/var/wili/db.sqlite3')
    node.logger.info('start')
    try:
        rclpy.spin(node)
    except KeyboardInterrupt: print('')
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == "__main__":
    main()