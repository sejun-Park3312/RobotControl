from ClassFiles.RobotControlFiles.RobotController_SDK import RobotController_SDK
from ClassFiles.RobotControlFiles.PathMaker import PathMaker
import numpy as np
from DSR_ROBOT import *

PM = PathMaker()

l = 45
r = l * np.sin(np.pi/4)
kc = np.cos(np.deg2rad(15))
ks = np.sin(np.deg2rad(15))

hc = np.cos(np.deg2rad(67.5))
hs = np.sin(np.deg2rad(67.5))

x = []
x.append([0,0,0,0,0,0])
x.append([l/2,l/2 * kc, l/2 * ks,0,0,0])

x.append([l + r * hc, r * hs * kc, r * hs * ks,0,0,0])
x.append([l + r, 0,0,0,0,-135])

x.append([l + r * hc, -r * hs * kc, -r * hs * ks,0,0,-135])
x.append([l/2,-l/2 * kc, -l/2 * ks,0,0,-270])

x.append([-l/2,l/2 * kc, l/2 * ks,0,0,-270])

x.append([-(l + r * hc), r * hs * kc, r * hs * ks,0,0,-270])
x.append([-(l + r), 0,0,0,0,-135])

x.append([-(l + r * hc), -r * hs * kc, -r * hs * ks,0,0,-135])
x.append([-l/2,-l/2 * kc, -l/2 * ks,0,0,0])

x.append([0,0,0,0,0,0])

PM.plot_3d_points(x, True)

np_x = np.array(x, dtype = float)
np_dx = np.diff(np_x, axis = 0)

dx = np_dx.tolist()

b_list = []
Radius = 1

i = 0
px = posx(dx[i][0], dx[i][1], dx[i][2], dx[i][3], dx[i][4], dx[i][5])
pb = posb(DR_LINE, px, radius = Radius)
b_list.append(pb)

i = 1
px1 = posx(dx[i][0], dx[i][1], dx[i][2], dx[i][3], dx[i][4], dx[i][5])
px2 = posx(dx[i+1][0], dx[i+1][1], dx[i+1][2], dx[i+1][3], dx[i+1][4], dx[i+1][5])
pb = posb(DR_CIRCLE, px1, px2, radius = 1)
b_list.append(pb)

i = 3
px1 = posx(dx[i][0], dx[i][1], dx[i][2], dx[i][3], dx[i][4], dx[i][5])
px2 = posx(dx[i+1][0], dx[i+1][1], dx[i+1][2], dx[i+1][3], dx[i+1][4], dx[i+1][5])
pb = posb(DR_CIRCLE, px1, px2, radius = Radius)
b_list.append(pb)

i = 5
px = posx(dx[i][0], dx[i][1], dx[i][2], dx[i][3], dx[i][4], dx[i][5])
pb = posb(DR_LINE, px, radius = Radius)
b_list.append(pb)

i = 6
px1 = posx(dx[i][0], dx[i][1], dx[i][2], dx[i][3], dx[i][4], dx[i][5])
px2 = posx(dx[i+1][0], dx[i+1][1], dx[i+1][2], dx[i+1][3], dx[i+1][4], dx[i+1][5])
pb = posb(DR_CIRCLE, px1, px2, radius = Radius)
b_list.append(pb)

i = 8
px1 = posx(dx[i][0], dx[i][1], dx[i][2], dx[i][3], dx[i][4], dx[i][5])
px2 = posx(dx[i+1][0], dx[i+1][1], dx[i+1][2], dx[i+1][3], dx[i+1][4], dx[i+1][5])
pb = posb(DR_CIRCLE, px1, px2, radius = Radius)
b_list.append(pb)

i = 10
px = posx(dx[i][0], dx[i][1], dx[i][2], dx[i][3], dx[i][4], dx[i][5])
pb = posb(DR_LINE, px, radius = Radius)
b_list.append(pb)


def main():
    return b_list

if __name__ == "__main__":
    main()





