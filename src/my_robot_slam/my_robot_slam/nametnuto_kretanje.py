#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray

broj_stepova_po_revoluciji = 200
poluprecnik_tocka_mm = 40
MM_PER_STEP = (2 * poluprecnik_tocka_mm * 3.14159) / broj_stepova_po_revoluciji

class NametnutoKretanjeNode(Node):
    def __init__(self):
        super().__init__("nametnuto_kretanje")
        self.get_logger().info("Čvor za nametnuto kretanje je pokrenut.")
        # Početne koordinate robota u milimetrima
        self.trenutno_x = 440 * 3 #mm od pocetnog polozaja na poligonu, a to je presek fugni dve plocice u ucionici 
        self.trenutno_y = 440    

        # Subscriber za ciljne koordinate
        self.kretanje_sub = self.create_subscription(
            Int32MultiArray,
            '/podaci_kretanja',
            self.callback,
            10)
        
        # Publisher za korake (stepove) motora
        self.kretanje_pub = self.create_publisher(Int32MultiArray, '/podaci_motion_planner', 10)

    def callback(self, msg):
        # Provera da li poruka ima tačno X i Y koordinatu
        if len(msg.data) < 2:
            self.get_logger().warn("Primljeni podaci nemaju dovoljno elemenata (očekuje se [X, Y]).")
            return

        ciljno_x = msg.data[0]
        ciljno_y = msg.data[1]


        maks_granica = 440 * 4
        ciljno_x = max(0, min(ciljno_x, maks_granica))
        ciljno_y = max(0, min(ciljno_y, maks_granica))

        step_x, step_y = self.izracunaj_naredbe(ciljno_x, ciljno_y)

        izlazna_poruka = Int32MultiArray()
        izlazna_poruka.data = [step_x, step_y]
        self.kretanje_pub.publish(izlazna_poruka)

        # Nakon što se proračun izvrši, ažuriramo trenutnu poziciju robota
        # (Pretpostavljamo da će motori uspešno izvršiti ove korake)
        self.trenutno_x = ciljno_x
        self.trenutno_y = ciljno_y

    def izracunaj_naredbe(self, cilj_x, cilj_y):
        # Izračunavamo razliku u milimetrima (može biti pozitivna ili negativna)
        razlika_x = cilj_x - self.trenutno_x
        razlika_y = cilj_y - self.trenutno_y

        # Pretvaramo milimetre u korake (deljenjem sa MM_PER_STEP)
        # Koristimo int() jer stepovi moraju biti celi brojevi
        step_x = int(razlika_x / MM_PER_STEP)
        step_y = int(razlika_y / MM_PER_STEP)

        return step_x, step_y

def main(args=None):    
    rclpy.init(args=args)
    node = NametnutoKretanjeNode()
    node.get_logger().info("Nametnuto kretanje je uspešno pokrenuto!")
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()