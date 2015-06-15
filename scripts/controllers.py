'''
	Developed by Samuel Tanner Lindemer 2015
	Le Laboaratoire des Signaux et Systemes
	Centrale-Supelec Universite, Gif-sur-Yvette, France
	samuel.lindemer@lss.supelec.fr
'''

from scipy.misc import factorial
import numpy as np

class Controller():
	def __init__(self):
		# physical constants
		self.m = 0.53				# total mass [kg]
		self.g = 9.81				# acceleration of gravity [m/s^2]
		self.Ix = 6.228e-3			# longitudinal inertia [kgm^2]
		self.Iy = 6.228e-3			# lateral inertia [kgm^2]
		self.Iz = 1.121e-3			# vertial inertia [kgm^2]

		# actuator values
		self.u  = np.array([0.0, 0.0, 0.0, 0.0])		# actual
		self.ur = np.array([0.0, 0.0, 0.0, 0.0])		# reference

class FlatControlLaw(Controller):
	def __init__(self):
		Controller.__init__(self)
		self.e = np.array([ 	[0.0, 0.0],	# ex,   dex
					[0.0, 0.0],	# ey,   dey
				 	[0.0, 0.0],	# ex,   dez
					[0.0, 0.0] ])	# epsi, depsi
		'''
		(s+s1)(s+s2)(s+s3)(s+s4) = s**4 + (s1+s2+s3+s4)*s**3 +
		                           (s1*s2+s1*s3+s1*s4+s2*s3+s2*s4+s3*s4)*s**2 +
		                           (s1*s2*s3+s1*s2*s4+s1*s3*s4+s2*s3*s4)*s
		                           (s1*s2*s3*s4) 
		'''
		self.lamz = np.array([30, 30])
		s1 = 12; s2 = 15; s3 = 13; s4 = 20
		self.lamxy = np.array([	s1*s2*s3*s4,
					s1*s2*s3+s1*s2*s4+s1*s3*s4+s2*s3*s4,
					s1*s2+s1*s3+s1*s4+s2*s3+s2*s4+s3*s4,
					s1+s2+s3+s4	])
		self.lampsi = np.array([125, 125])
		
	def __call__(self, y, r):
		self.e[0][0] = y[0]  - r[0][0];		self.e[0][1] = y[1]  - r[0][1]
		self.e[1][0] = y[2]  - r[1][0];		self.e[1][1] = y[3]  - r[1][1]
		self.e[2][0] = y[4]  - r[2][0];		self.e[2][1] = y[5]  - r[2][1]
		self.e[3][0] = y[10] - r[3][0];		self.e[3][1] = y[11] - r[3][1]

		self.u[0]	=   self.m * (self.g + r[2][2] - self.lamz[0] * self.e[2][0] - self.lamz[1] * self.e[2][1])
		d2ez 		=  ( self.u[0] / self.m) - self.g - r[2][2]
		du1  		=   self.m * (r[2][3] - self.lamz[0] * self.e[2][1] - self.lamz[1] * d2ez)
		d3ez 		=   du1 / self.m - r[2][3]
		d2u1 		=   self.m * (r[2][4] - self.lamz[0] * d2ez - self.lamz[1] * d3ez)  
		d2ey 		= -(y[6] * self.u[0]) / self.m - r[1][2] 
		d3ey 		= -(y[7] * self.u[0]  + y[6] * du1) / self.m - r[1][3]
		d2ex 		=  (y[8] * self.u[0]) / self.m - r[0][2]  
		d3ex 		=  (y[9] * self.u[0]  + y[8] * du1) / self.m - r[0][3]
		self.u[1]   	= (-self.Ix / self.u[0]) * ( 2 * y[7] * du1 + y[6] * d2u1 + self.m * r[1][4] - self.m * (self.lamxy[0] * self.e[1][0] + self.lamxy[1] * self.e[1][1] + self.lamxy[2] * d2ey + self.lamxy[3] * d3ey))
		self.u[2]	= ( self.Iy / self.u[0]) * (-2 * y[9] * du1 - y[8] * d2u1 + self.m * r[0][4] - self.m * (self.lamxy[0] * self.e[0][0] + self.lamxy[1] * self.e[0][1] + self.lamxy[2] * d2ex + self.lamxy[3] * d3ex))
		self.u[3]   	=   self.Iz * (r[3][2] - self.lampsi[0] * self.e[3][0] - self.lampsi[1] * self.e[3][1])

		self.ur[0]	=   self.m * (self.g + r[2][2])
		du1r		=   self.m *  r[2][3]
		d2u1r		=   self.m *  r[2][4]   
		phir		=   self.m *  r[1][2] / self.ur[0]
		dphir		=   self.m * (r[1][3] * self.ur[0] - r[1][2] * du1r) / self.ur[0]**2
		thetar		=   self.m *  r[0][2] / self.ur[0]
		dthetar		=   self.m * (r[0][3] * self.ur[0] - r[0][2] * du1r) / self.ur[0]**2
		self.ur[1]	= (-self.Ix / self.ur[0]) * ( 2 * dphir   * du1r + phir   * d2u1r + self.m * r[1][4])                  
		self.ur[2]	= ( self.Iy / self.ur[0]) * (-2 * dthetar * du1r - thetar * d2u1r + self.m * r[0][4]) 
		self.ur[3]	=   self.Iz * r[3][2]

		return self.u, self.ur

class Pid():
	def __init__(self, Kp, Ki, Kd, lim=None):
		self.Kp = Kp
		self.Ki = Ki
		self.Kd = Kd
		self.lim = lim
		self.integrator_lim = (-20, 20)
		self.integrator = 0
		self.derivator = 0
		self.dt = 0.01
		
	def set_reference(self, r):
		self.r = r
		
	def get_u(self, y):
		e = self.r - y
		P = self.Kp * e
		D = self.Kd * (e - self.derivator) / self.dt
		self.derivator = e
		self.integrator += e
		if   self.integrator < self.integrator_lim[0]: self.integrator = self.integrator_lim[0]
		elif self.integrator > self.integrator_lim[1]: self.integrator = self.integrator_lim[1]
		I = self.integrator * self.Ki
		PID = P + I + D
		if self.lim and PID < self.lim[0]: PID = self.lim[0]
		if self.lim and PID > self.lim[1]: PID = self.lim[1]
		return PID

class DiscretePid(Controller):
	def __init__(self):
		Controller.__init__(self)
		self.pids = dict(	u1A=Pid(2.0*self.m*self.g, 0.5*self.m*self.g, 0.0, lim=(0, 10)),
					u2A=Pid(20.0*self.Ix, 5.0*self.Ix, 0.0),
					u3A=Pid(20.0*self.Iy, 5.0*self.Iy, 0.0),
					u4A=Pid(20.0*self.Iz, 5.0*self.Iz, 0.0)					)
		for pid in self.pids: self.pids[pid].set_reference(0.0)

	def __call__(self, y, keys):
		if   keys['up']    == True:	self.u[0] = 10.0
		elif keys['down']  == True:	self.u[0] =  0.0
		else: 				self.u[0] = self.pids['u1A'].get_u(y[5])
		if   keys['l']     == True:	self.u[1] =  0.05
		elif keys['j']     == True:	self.u[1] = -0.05
		else:				self.u[1] = self.pids['u2A'].get_u(y[7])
		if   keys['i']     == True:	self.u[2] =  0.05
		elif keys['k']     == True:	self.u[2] = -0.05
		else:				self.u[2] = self.pids['u3A'].get_u(y[9])
		if   keys['left']  == True:	self.u[3] =  0.025
		elif keys['right'] == True:	self.u[3] = -0.025
		else:				self.u[3] = self.pids['u4A'].get_u(y[11])

		return self.u

class CascadeInteractive(Controller):
	def __init__(self):
		Controller.__init__(self)
		self.pids = dict(	u1A=Pid(2.0*self.m*self.g, 0.5*self.m*self.g, 0.0, lim=(0, 10)),
					u2A=Pid(20.0*self.Ix, 0.0, 0.0),
					u2B=Pid(20.0, 0.0, 0.0, lim=(-np.pi, np.pi)),
					u2C=Pid(4.0/self.g, 0.0, 0.0, lim=(-np.pi/4, np.pi/4)),
					u3A=Pid(20.0*self.Iy, 0.0, 0.0),
					u3B=Pid(20.0, 0.0, 0.0, lim=(-np.pi, np.pi)),
					u3C=Pid(4.0/self.g, 0.0, 0.0, lim=(-np.pi/4, np.pi/4)),
					u4A=Pid(20.0*self.Iz, 5.0*self.Iz, 0.0, lim=(0, 10))			)

	def __call__(self, y, keys):
		if   keys['up']    == True:	self.pids['u1A'].set_reference( 3.0)
		elif keys['down']  == True:	self.pids['u1A'].set_reference(-3.0)
		else: 				self.pids['u1A'].set_reference( 0.0)
		self.u[0] = self.pids['u1A'].get_u(y[5])

		gamma = None
		if keys['i']:
			gamma = y[10]
			if keys['j']: gamma += np.pi/4
			if keys['l']: gamma -= np.pi/4
		elif keys['k']:
			gamma = y[10] + np.pi
			if keys['j']: gamma -= np.pi/4
			if keys['l']: gamma += np.pi/4
		elif keys['j']:	gamma = y[10] + np.pi/2
		elif keys['l']: gamma = y[10] - np.pi/2

		if gamma is not None:
			self.pids['u2C'].set_reference(10.0 * np.sin(gamma))
			self.pids['u3C'].set_reference(10.0 * np.cos(gamma))
		else:
			self.pids['u2C'].set_reference(0.0)
			self.pids['u3C'].set_reference(0.0)

		self.pids['u2B'].set_reference(self.pids['u2C'].get_u(y[3]) * -1.0)
		self.pids['u2A'].set_reference(self.pids['u2B'].get_u(y[6]))
		self.u[1] =		       self.pids['u2A'].get_u(y[7])
		self.pids['u3B'].set_reference(self.pids['u3C'].get_u(y[1]))
		self.pids['u3A'].set_reference(self.pids['u3B'].get_u(y[8]))
		self.u[2] =		       self.pids['u3A'].get_u(y[9])

		if   keys['left']  == True:	self.pids['u4A'].set_reference( np.pi)
		elif keys['right'] == True:	self.pids['u4A'].set_reference(-np.pi)
		else:				self.pids['u4A'].set_reference( 0.0)
		self.u[3] = 			self.pids['u4A'].get_u(y[11])

		return self.u

class CascadeTracking(Controller):
	def __init__(self):
		Controller.__init__(self)
		self.pids = dict(	u1A=Pid(2.0, 0.0, 0.0, lim=(0, 10)),
					u1B=Pid(2.0*self.m*self.g, 0.0, 0.0),
					u2A=Pid(20.0*self.Ix, 0.0, 0.0),
					u2B=Pid(20.0, 0.0, 0.0, lim=(-np.pi, np.pi)),
					u2C=Pid(4.0/self.g, 0.0, 0.0, lim=(-np.pi/4, np.pi/4)),
					u2D=Pid(4.0, 0.0, 0.0),
					u3A=Pid(20.0*self.Iy, 0.0, 0.0),
					u3B=Pid(20.0, 0.0, 0.0, lim=(-np.pi, np.pi)),
					u3C=Pid(4.0/self.g, 0.0, 0.0, lim=(-np.pi/4, np.pi/4)),
					u3D=Pid(4.0, 0.0, 0.0),
					u4A=Pid(20.0*self.Iz, 5.0*self.Iz, 0.0, lim=(0, 10)),
					u4B=Pid(2.0, 0.0, 0.0)							)

	def __call__(self, y, r):
		self.pids['u1B'].set_reference(r[2][0])
		self.pids['u2D'].set_reference(r[1][0])
		self.pids['u3D'].set_reference(r[0][0])
		self.pids['u4B'].set_reference(r[3][0])
		self.pids['u1A'].set_reference(self.pids['u1B'].get_u(y[4]))
		self.u[0] = 		       self.pids['u1A'].get_u(y[5])
		self.pids['u2C'].set_reference(self.pids['u2D'].get_u(y[2]))
		self.pids['u2B'].set_reference(self.pids['u2C'].get_u(y[3]) * -1.0)
		self.pids['u2A'].set_reference(self.pids['u2B'].get_u(y[6]))
		self.u[1] =		       self.pids['u2A'].get_u(y[7])
		self.pids['u3C'].set_reference(self.pids['u3D'].get_u(y[0]))
		self.pids['u3B'].set_reference(self.pids['u3C'].get_u(y[1]))
		self.pids['u3A'].set_reference(self.pids['u3B'].get_u(y[8]))
		self.u[2] =		       self.pids['u3A'].get_u(y[9])
		self.pids['u4A'].set_reference(self.pids['u4B'].get_u(y[10]))
		self.u[3] = 		       self.pids['u4A'].get_u(y[11])

		ur = [	self.pids['u1A'].r,
			self.pids['u2A'].r,
			self.pids['u3A'].r,
			self.pids['u4A'].r	]

		return self.u, ur
