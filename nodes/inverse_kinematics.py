import sys
import numpy as np
from forward_kinematics import *

A1 = L23
A2 = L34
A3 = L45 + L5TCP

YAW_X = BASE[0]
YAW_Y = BASE[1]
SH_Z = BASE[2] + L12

def rad_to_servo(q, i):
	return int(round(CENTER + q / (SIGN[i] * SCALE)))
	
def ik(x, y, z, pitch_deg=90.0, elbow=-1):
	dx = x - YAW_X
	dy = y - YAW_Y
	phi = np.arctan2(dy, dx)
	q1 = -phi
	r = np.hypot(dx, dy)
	Th = np.deg2rad(pitch_deg)
	wr = r - A3 * np.sin(Th)
	wz = z - A3 * np.cos(Th)
	dr = wr
	dz = wz - SH_Z
	D = np.hypot(dr, dz)
	print('D=', round(D,4), 'reach=', round(A1+A2,4), 'links=', round(A1,4), round(A2,4), round(A3,4))
	if D > A1 + A2 + 0.02 or D < abs(A1 - A2) - 0.02:
		return None
	cb = (D*D - A1*A1 - A2*A2) / (2 * A1 * A2)
	cb = max(-1.0, min(1.0, cb))
	bend = elbow * np.arccos(cb)
	rho = np.arctan2(dr, dz)
	th2 = rho - np.arctan2(A2 * np.sin(bend), A1 + A2 * np.cos(bend))
	q2 = th2
	q3 = bend
	q4 = Th - (th2 + bend)
	return [q1, q2, q3, q4, 0.0]
	
def main():
	if len(sys.argv) < 4:
		print('usage: ik.py X Y Z [pitch_deg] [elbow +1/-1] (meters, base frame)')
		return
	x, y, z = [float(v) for v in sys.argv[1:4]]
	pitch = float(sys.argv[4]) if len(sys.argv) > 4 else 90.0
	elbow = int(sys.argv[5]) if len(sys.argv) > 5 else -1
	res = ik(x, y, z, pitch, elbow)
	if res is None:
		print('target out of reach (bring it closer or change pitch)')
		return
		
	servos = [rad_to_servo(res[i], i) for i in range(5)]
	tip = fk(res)
	err = np.linalg.norm(tip - np.array([x, y, z])) * 1000
	print('target x=%.3f y=%.3f z=%.3f m pitch=%.0f elbow=%d' % (x, y, z, pitch, elbow))
	print('angles ', [round(np.rad2deg(a), 1) for a in res], 'deg')
	print('servos ', servos)
	print('FK check x=%.3f y=%.3f z=%.3f m err=%.1f mm' % (tip[0], tip[1], tip[2], err))
	if min(servos) < 0 or max(servos) > 1000:
		print('WARNING: a servo is outside 0-1000 -- try elbow %+d or a different pitch' % (-elbow))
		
if __name__ == '__main__':
	main()
