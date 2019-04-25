import sys
import time
from simulation.vrepSim.Bioloid import Bioloid

# ############################################ POSTUREs ###############################################

INITIAL_POS = [336, 687, 298, 724, 412, 611, 355, 664, 491, 530, 394, 625, 278, 743, 616, 405, 490, 530]

STAND_POS = [512] * 18
STAND_POS[6] = 512 - 151
STAND_POS[7] = 512 + 151  # 151 = 45 degrees

# #####################################################################################################


robot = Bioloid()

t = time.time()
robot.set_degree(1, STAND_POS)
t2 = time.time()
print(robot.read_motor())
print((t - t2) * 1000)

# while time.time()-t < 60.0:
#
