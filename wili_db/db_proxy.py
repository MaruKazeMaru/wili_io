import sqlite3
import rclpy
from rclpy.node import Node
from wili_msgs.srv import GetTrProbMat

class DBProxy(Node):
    def __init__(self, db_path:str):
        super().__init__("db_proxy")
        self.db_conn = sqlite3.connect(db_path)
        self.db_cur = self.db_conn.cursor()
        self.srv_tr_mat = self.create_service(GetTrProbMat, "get_tr_prob", self.get_tr_prob_mat)

    def __del__(self):
        self.db_conn.close()


    def get_tr_prob_mat(self, req:GetTrProbMat.Request, res:GetTrProbMat.Response):
        self.db_cur.execute('SELECT elem FROM tr_prob ORDER BY from_motion, to_motion')
        res.data = self.db_cur.fetchall()
        return res


def main():
    node = DBProxy()
    rclpy.spin(node)


if __name__ == "__main__":
    main()