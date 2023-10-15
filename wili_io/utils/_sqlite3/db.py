#!/usr/bin/env python3

# SPDX-FileCopyrightText: 2023 ShinagwaKazemaru
# SPDX-License-Identifier: MIT License

import sqlite3

class DBManager:
    def __init__(self, db_path:str):
        self.conn = sqlite3.connect(db_path, check_same_thread=False)
        self.cur = self.conn.cursor()


    def count_motion(self) -> int:
        # get number of motion
        self.cur.execute('SELECT COUNT(id) FROM motion')
        return self.cur.fetchone()[0]
    

    def select_fetch_init_prob(self) -> list[float]:
        # get transition probabilities
        self.cur.execute('SELECT data FROM init_prob ORDER BY motion')
        return [r[0] for r in self.cur.fetchall()]


    def select_fetch_tr_prob(self) -> list[float]:
        # get transition probabilities
        self.cur.execute('SELECT data FROM tr_prob ORDER BY from_motion, to_motion')
        return [r[0] for r in self.cur.fetchall()]


    def select_fetch_gaussian(self) -> list[list[float]]:
        # get gaussian of each motion
        self.cur.execute( \
            'SELECT ' \
            'avr_x, avr_y, ' \
            'covar_xx, covar_xy, covar_yy ' \
            'FROM gaussian ORDER BY motion' \
        )
        return self.cur.fetchall()