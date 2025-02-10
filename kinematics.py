import math

# Inputs: x (forward/backward), y (height), L1 (arm 1 length), L2 (arm 2 length)
# Outputs: theta1 (angle)
def forward(x,y,L1,L2):
    return math.acos((x**2 + y**2 - L1**2 - L2**2)/(2*L1*L2))

# Inputs: x (forward/backward), y (height), L1 (arm 1 length), L2 (arm 2 length), theta1 (angle)
# Outputs: theta2 (angle)
def inverse(x,y,L1,L2,theta1):
    return math.atan2(y, x) - math.atan2(L2*math.sin(theta1), L1 + L2*math.cos(theta1))

