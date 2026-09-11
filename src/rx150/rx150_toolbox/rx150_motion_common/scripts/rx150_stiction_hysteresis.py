"""Tiếp cận CÙNG tư thế từ hai phía -> sai số xác lập có đảo dấu không?

Đảo dấu = sai số nằm trong dải chết ma sát tĩnh, KHÔNG phải thiên lệch mô hình
trọng lực. Khi đó tuning gravity thêm nữa vô nghĩa; đòn bẩy là friction FF.
"""
import math, time
import numpy as np, rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray

ARM = ["waist","shoulder","elbow","wrist_angle","wrist_rotate"]
TARGETS = {
    "home":      [0.0, -1.260, 1.381, -0.120, 0.0],
    "reach_far": [0.0, -0.90,  1.00,   0.30,  0.0],
    "up_high":   [0.0, -1.50,  0.70,  -0.20,  0.0],
}
MARGIN = 0.25   # rad, cùng approach-margin của rx150_gravity_id.py

class H(Node):
    def __init__(self):
        super().__init__("hyst")
        self.err=None
        self.create_subscription(JointState,"/rx150/hac/error",
                                 lambda m:setattr(self,"err",m), qos_profile_sensor_data)
        self.pub=self.create_publisher(Float64MultiArray,"/rx150/hac/setpoint",10)
    def hold(self,q,s):
        m=Float64MultiArray(); m.data=[float(v) for v in q]
        e=time.monotonic()+s
        while time.monotonic()<e:
            self.pub.publish(m); rclpy.spin_once(self,timeout_sec=0.02)
    def sample(self,q,s=2.0):
        m=Float64MultiArray(); m.data=[float(v) for v in q]
        acc=[]; e=time.monotonic()+s
        while time.monotonic()<e:
            self.pub.publish(m); rclpy.spin_once(self,timeout_sec=0.02)
            if self.err:
                idx={n:i for i,n in enumerate(self.err.name)}
                acc.append([self.err.position[idx[j]] for j in ARM])
        return np.degrees(np.mean(np.array(acc),axis=0))

rclpy.init(); h=H()
h.hold(TARGETS["home"], 6.0)
print(f"{'tư thế':<11}{'hướng':<7}" + "".join(f"{j[:5]:>9}" for j in ARM))
band={}
for name,q in TARGETS.items():
    res={}
    for tag,sgn in (("từ +",+1.0),("từ -",-1.0)):
        appr=list(q)
        for i in (1,2,3): appr[i]=q[i]+sgn*MARGIN
        h.hold(appr,5.0); h.hold(q,4.0)
        res[tag]=h.sample(q)
        print(f"{name:<11}{tag:<7}" + "".join(f"{v:>+9.2f}" for v in res[tag]))
    spread=res["từ +"]-res["từ -"]
    band[name]=spread
    print(f"{'':<11}{'CHÊNH':<7}" + "".join(f"{v:>+9.2f}" for v in spread))
h.hold([0.0,-1.80,1.55,0.8,0.0], 8.0)
arr=np.array(list(band.values()))
print("\nchênh lệch |từ + − từ −| trung bình theo khớp (độ):")
print("  " + "  ".join(f"{j[:5]}={abs(arr[:,i]).mean():5.2f}" for i,j in enumerate(ARM)))
h.destroy_node(); rclpy.shutdown()
