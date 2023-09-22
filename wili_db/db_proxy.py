import sqlite3
import rclpy
from rclpy.parameter import Parameter
from rclpy.node import Node
from wili_msgs.msg import HMM
from wili_msgs.srv import GetHMM

class DBProxy(Node):
    def __init__(self, db_path:str):
        super().__init__("db_proxy")

        self.db_conn = sqlite3.connect(db_path)
        self.db_cur = self.db_conn.cursor()
        self.srv_tr_mat = self.create_service(GetHMM, "get_hmm", self.get_hmm)


    def destroy_node(self):
        self.db_conn.close()
        super().destroy_node()


    def get_hmm(self, req:GetHMM.Request, res:GetHMM.Response):
        self.db_cur.execute('SELECT elem FROM tr_prob ORDER BY from_motion, to_motion')
        hmm = HMM()
        hmm.tr_prob = [f[0] for f in self.db_cur.fetchall()]
        res.hmm = hmm
        return res


def main():
    rclpy.init()
    node = DBProxy('../test.db')
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()