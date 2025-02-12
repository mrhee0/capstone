import math

class Kinematics:
    def __init__(self, L1, L2):
        self.L1 = L1
        self.L2 = L2

    # Inputs: x (forward/backward), y (height), L1 (arm 1 length), L2 (arm 2 length)
    # Outputs: theta1 (angle)
    def forward(self, x, y):
        num = x**2 + y**2 - self.L1**2 - self.L2**2
        den = 2 * self.L1 * self.L2
        return math.acos(num / den)

    # Inputs: x (forward/backward), y (height), L1 (arm 1 length), L2 (arm 2 length), theta1 (angle)
    # Outputs: theta2 (angle)
    def inverse(self, x, y, theta1):
        first = math.atan2(y, x)
        second = math.atan2(self.L2 * math.sin(theta1), self.L1 + self.L2 * math.cos(theta1))
        return first - second
