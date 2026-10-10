import sys
import numpy as np

BASE = (0.0251328065010765, 0.0, 0.156176218742268)
L12 = 0.0338648012164686
L23 = 0.129416446394797
L34 = 0.129444631186569
L45 = 0.0544833339503674
L5TCP = 0.0854388

CENTER = 500
SCALE = np.deg2rad(240.0 / 1000.0)
SIGN = [1, -1, -1, -1, 1]

def servo_to_rad(servo, i):
	return SIGN[i] * (servo - CENTER) * SCALE
	
def Rz(a):
	c, s = np.cos(a), np.sin(a)
	return np.array([[c, -s, 0, 0], [s, c, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]])

def Ry(a):
	c, s = np.cos(a), np.sin(a)
	return np.array([[c, 0, s, 0], [0, 1, 0, 0], [-s, 0, c, 0], [0, 0, 0, 1]])
	
def T(x, y, z):
	m = np.eye(4)
	m[0, 3] = x
	m[1, 3] = y
	m[2, 3] = z
	return m
	
def joint_points(q):
	pts = {}
	M = T(*BASE) @ Rz(-q[0]) @ T(0, 0, L12)
	pts['shoulder'] = M[:3, 3].copy()
	M = M @ Ry(q[1]) @ T(0, 0, L23)
	pts['elbow'] = M[:3, 3].copy()
	M = M @ Ry(q[2]) @ T(0, 0, L34)
	pts['wrist'] = M[:3, 3].copy()
	M = M @ Ry(q[3]) @ T(0, 0, L45) @ Rz(-q[4]) @ T(0, 0, L5TCP)
	pts['tip'] = M[:3, 3].copy()
	return pts
	
def fk(q):
	return joint_points(q)['tip']
	
def main():
	servos = [int(x) for x in sys.argv[1:6]] if len(sys.argv) > 1 else [500] * 5
	while len(servos) < 5:
		servos.append(500)
	q = [servo_to_rad(servos[i], i) for i in range(5)]
	pts = joint_points(q)
	print(f'servos {servos}')
	print(f'angles {[round(np.rad2deg(a), 1) for a in q]} deg')
	for k in ['shoulder', 'elbow', 'wrist', 'tip']:
		p = pts[k]
		print(f' {k:9s} x={p[0]*1000:+7.1f} y={p[1]*1000:+7.1f} z={p[2]*1000:+7.1f} mm')
		
if __name__ == '__main__':
	main()
