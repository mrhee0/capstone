import math

class Kinematics:
    # L1 (arm 1 length), L2 (arm 2 length)
    def __init__(self, L1, L2):
        self.L1 = L1
        self.L2 = L2

    # Inputs: x (forward/backward), y (height)
    def inverse_kinematics(self, x, y):
        x = -x
        d = math.sqrt(x**2 + y**2)
        if d > (self.L1 + self.L2):
            return None
        
        # Law of cosines
        cos_angle2 = (x**2 + y**2 - self.L1**2 - self.L2**2) / (2 * self.L1 * self.L2)
        angle2 = math.acos(max(min(cos_angle2, 1), -1))
        
        # Law of sines
        k1 = self.L1 + self.L2 * math.cos(angle2)
        k2 = self.L2 * math.sin(angle2)
        angle1 = math.atan2(y, x) - math.atan2(k2, k1)
        
        return math.degrees(angle1), math.degrees(angle2)

# k = Kinematics(20,20)
# t1,t2=k.inverse_kinematics(20,15)
# print(t1)
# print(t2)
