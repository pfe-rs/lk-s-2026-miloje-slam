# 100% NE-AI KOD NAPISAN RADI RAZUMEVANJA

import time
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
import os

class Baterija(Node):
    def __init__(self, kriticni_napon):
        super().__init__('Monitor baterije')
        self.kriticni_napon = kriticni_napon
        self.brojac_tikova = 0
        self.potrebno_tikova = 5

        self.subscription = self.create_subscription(
            Float32,
            'napon_baterije', # IME PORUKE
            self.citanje_napona,
            10
        )

    def citanje_napona(self, msg):
        trenutni_napon = msg.data

        if (trenutni_napon < self.kriticni_napon):
            self.brojac_tikova = self.brojac_tikova + 1

            if(self.brojac_tikova >= self.potrebno_tikova):
                self.UbijSe()

        if(trenutni_napon > self.kriticni_napon):
            self.brojac_tikova = 0

    def UbijSe(self):
        self.get_logger().error("!!! CRITICAL LIPO VOLTAGE REACHED. SHUTTING DOWN SYSTEM NOW !!!")
        
        time.sleep(1)
        os.system("sudo shutdown now")

def main(args = None):
    rclpy.init(args = args) # Kriticni napon je 9.3 volta
    node = Baterija(9.3)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
    

if __name__ == '__main__':
    main()