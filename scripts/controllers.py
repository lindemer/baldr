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
		self.e = np.array([ [0.0, 0.0],		# ex,   dex
						 	[0.0, 0.0],		# ey,   dey
						  	[0.0, 0.0],		# ex,   dez
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

		self.u[0]   =   self.m * (self.g + r[2][2] - self.lamz[0] * self.e[2][0] - self.lamz[1] * self.e[2][1])
		d2ez 		=  ( self.u[0] / self.m) - self.g - r[2][2]
		du1  		=   self.m * (r[2][3] - self.lamz[0] * self.e[2][1] - self.lamz[1] * d2ez)
		d3ez 		=   du1 / self.m - r[2][3]
		d2u1 		=   self.m * (r[2][4] - self.lamz[0] * d2ez - self.lamz[1] * d3ez)  
		d2ey 		= -(y[6] * self.u[0]) / self.m - r[1][2] 
		d3ey 		= -(y[7] * self.u[0]  + y[6] * du1) / self.m - r[1][3]
		d2ex 		=  (y[8] * self.u[0]) / self.m - r[0][2]  
		d3ex 		=  (y[9] * self.u[0]  + y[8] * du1) / self.m - r[0][3]
		self.u[1]   = (-self.Ix / self.u[0]) * ( 2 * y[7] * du1 + y[6] * d2u1 + self.m * r[1][4] - self.m * (self.lamxy[0] * self.e[1][0] + self.lamxy[1] * self.e[1][1] + self.lamxy[2] * d2ey + self.lamxy[3] * d3ey))
		self.u[2]   = ( self.Iy / self.u[0]) * (-2 * y[9] * du1 - y[8] * d2u1 + self.m * r[0][4] - self.m * (self.lamxy[0] * self.e[0][0] + self.lamxy[1] * self.e[0][1] + self.lamxy[2] * d2ex + self.lamxy[3] * d3ex))
		self.u[3]   =   self.Iz * (r[3][2] - self.lampsi[0] * self.e[3][0] - self.lampsi[1] * self.e[3][1])

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

class DiscretePid(Controller):
	def __init__(self):
		Controller.__init__(self)
		self.params = { 'dz':		{'Kp':2.0, 'Ki':0.5, 'Kd':3.0,
						'int_min':-20, 'int_max':20,
						'ref':0, 'integrator':0, 'derivator':0,
						'hi_control':10.00, 'lo_control':0.00,
						'PID_max':10.00, 'PID_min':0.00,
						'gain_multiplier':self.m*self.g		},
			      	'dphi':		{'Kp':2.0, 'Ki':0.5, 'Kd':2.0,
						'int_min':-20, 'int_max':20,
						'ref':0, 'integrator':0, 'derivator':0,
						'hi_control':0.05, 'lo_control':-0.05,
						'PID_max':None, 'PID_min':None,
						'gain_multiplier':10*self.Ix		},
			    	'dtheta':	{'Kp':2.0, 'Ki':0.5, 'Kd':2.0,
						'int_min':-20, 'int_max':20,
						'ref':0, 'integrator':0, 'derivator':0,
						'hi_control':0.05, 'lo_control':-0.05,
						'PID_max':None, 'PID_min':None,
						'gain_multiplier':10*self.Iy		},
			      	'dpsi':		{'Kp':2.0, 'Ki':0.5, 'Kd':2.5,
						'int_min':-20, 'int_max':20,
						'ref':0, 'integrator':0, 'derivator':0,
						'hi_control':0.025, 'lo_control':-0.025,	
						'PID_max':None, 'PID_min':None,
						'gain_multiplier':10*self.Iz		}	}

		for par in self.params:
			self.params[par]['Kp'] *= self.params[par]['gain_multiplier']
			self.params[par]['Ki'] *= self.params[par]['gain_multiplier']
			self.params[par]['Kd'] *= self.params[par]['gain_multiplier']

	def __call__(self, y, keys):
		self.params['dz']['state']     = y[5]
		self.params['dphi']['state']   = y[7]
		self.params['dtheta']['state'] = y[9]
		self.params['dpsi']['state']   = y[11]
 
		if   keys['up']    == True:	self.u[0] = self.params['dz']['hi_control']
		elif keys['down']  == True:	self.u[0] = self.params['dz']['lo_control']
		else: 				self.u[0] = self.get_pid(self.params['dz'])
		if   keys['l']     == True:	self.u[1] = self.params['dphi']['hi_control']
		elif keys['j']     == True:	self.u[1] = self.params['dphi']['lo_control']
		else:				self.u[1] = self.get_pid(self.params['dphi'])
		if   keys['i']     == True:	self.u[2] = self.params['dtheta']['hi_control']
		elif keys['k']     == True:	self.u[2] = self.params['dtheta']['lo_control']
		else:				self.u[2] = self.get_pid(self.params['dtheta'])
		if   keys['left']  == True:	self.u[3] = self.params['dpsi']['hi_control']
		elif keys['right'] == True:	self.u[3] = self.params['dpsi']['lo_control']
		else:				self.u[3] = self.get_pid(self.params['dpsi'])	

		return self.u

	def get_pid(self, par):
		error = par['ref'] - par['state']
		P_val = par['Kp'] * error
		D_val = par['Kd'] * (error - par['derivator'])
		par['derivator'] = error

		par['integrator'] += error
		if   par['integrator'] > par['int_max']: par['integrator'] = par['int_max']
		elif par['integrator'] < par['int_min']: par['integrator'] = par['int_min']
		I_val = par['integrator'] * par['Ki']

		PID = P_val + I_val + D_val
		if par['PID_min'] and PID < par['PID_min']: PID = par['PID_min']
		if par['PID_max'] and PID > par['PID_max']: PID = par['PID_max']

		return PID

class CascadeInteractive(Controller):
	def __init__(self):
		Controller.__init__(self)

		self.pids = {}
		self.pids['u1A'] = dict(Kp=2.0, Ki=0.5, Kd=3.0,
					integrator_limit=(-20, 20),
					pid_limit=(0, 10),
					state=0, reference=0,
					derivator=0, integrator=0,
					gain_factor=self.m*self.g)
		self.pids['u3A'] = dict(Kp=2.0, Ki=0.0, Kd=2.0,
					integrator_limit=(-20, 20),
					pid_limit=None,
					state=0, reference=0,
					derivator=0, integrator=0,
					gain_factor=10*self.Ix)
		self.pids['u3B'] = dict(Kp=2.0, Ki=0.0, Kd=4.0,
					integrator_limit=(-20, 20),
					pid_limit=(-np.pi, np.pi),
					state=0, reference=0,
					derivator=0, integrator=0,
					gain_factor=10)
		self.pids['u3C'] = dict(Kp=4.0, Ki=0.0, Kd=2.0,
					integrator_limit=(-20, 20),
					pid_limit=(-np.pi/4, np.pi/4),
					state=0, reference=0,
					derivator=0, integrator=0,
					gain_factor=1/self.g)
		self.pids['u2A'] = dict(Kp=2.0, Ki=0.0, Kd=2.0,
					integrator_limit=(-20, 20),
					pid_limit=None,
					state=0, reference=0,
					derivator=0, integrator=0,
					gain_factor=10*self.Iy)
		self.pids['u2B'] = dict(Kp=2.0, Ki=0.0, Kd=4.0,
					integrator_limit=(-20, 20),
					pid_limit=(-np.pi, np.pi),
					state=0, reference=0,
					derivator=0, integrator=0,
					gain_factor=10)
		self.pids['u2C'] = dict(Kp=4.0, Ki=0.0, Kd=2.0,
					integrator_limit=(-20, 20),
					pid_limit=(-np.pi/4, np.pi/4),
					state=0, reference=0,
					derivator=0, integrator=0,
					gain_factor=1/self.g)
		self.pids['u4A'] = dict(Kp=2.0, Ki=0.5, Kd=2.5,
					integrator_limit=(-20, 20),
					pid_limit=None,
					state=0, reference=0,
					derivator=0, integrator=0,
					gain_factor=10*self.Iz)

		for pid in self.pids:
			self.pids[pid]['Kp'] *= self.pids[pid]['gain_factor']
			self.pids[pid]['Ki'] *= self.pids[pid]['gain_factor']
			self.pids[pid]['Kd'] *= self.pids[pid]['gain_factor']

	def __call__(self, y, keys):
		self.pids['u1A']['state'] = y[5]
		self.pids['u3A']['state'] = y[9]
		self.pids['u3B']['state'] = y[8]
		self.pids['u3C']['state'] = y[1]
		self.pids['u2A']['state'] = y[7]
		self.pids['u2B']['state'] = y[6]
		self.pids['u2C']['state'] = y[3]
		self.pids['u4A']['state'] = y[11]

		if   keys['up']    == True:	self.pids['u1A']['reference'] =  3.0
		elif keys['down']  == True:	self.pids['u1A']['reference'] = -3.0
		else: 				self.pids['u1A']['reference'] =  0.0
		self.u[0] = self.get_pid(self.pids['u1A'])

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
			self.pids['u2C']['reference'] = 10.0 * np.sin(gamma)
			self.pids['u3C']['reference'] = 10.0 * np.cos(gamma)
		else:
			self.pids['u2C']['reference'] = 0.0
			self.pids['u3C']['reference'] = 0.0

		self.pids['u2B']['reference'] = 	self.get_pid(self.pids['u2C']) * -1.0
		self.pids['u2A']['reference'] = 	self.get_pid(self.pids['u2B'])
		self.u[1] =				self.get_pid(self.pids['u2A'])
		self.pids['u3B']['reference'] = 	self.get_pid(self.pids['u3C'])
		self.pids['u3A']['reference'] =		self.get_pid(self.pids['u3B'])
		self.u[2] =				self.get_pid(self.pids['u3A'])

		if   keys['left']  == True:	self.pids['u4A']['reference'] =  np.pi
		elif keys['right'] == True:	self.pids['u4A']['reference'] = -np.pi
		else:				self.pids['u4A']['reference'] =  0.0
		self.u[3] = self.get_pid(self.pids['u4A'])

		return self.u

	def get_pid(self, pid):
		error = pid['reference'] - pid['state']
		P = pid['Kp'] *  error
		D = pid['Kd'] * (error - pid['derivator'])
		pid['derivator'] = error

		pid['integrator'] += error
		if   pid['integrator'] > pid['integrator_limit'][1]: pid['integrator'] = pid['integrator_limit'][1]
		elif pid['integrator'] < pid['integrator_limit'][0]: pid['integrator'] = pid['integrator_limit'][0]
		I = pid['integrator'] * pid['Ki']

		PID = P + I + D

		if pid['pid_limit']:
			if   PID < pid['pid_limit'][0]: PID = pid['pid_limit'][0]
			elif PID > pid['pid_limit'][1]: PID = pid['pid_limit'][1]

		return PID

class CascadeTracking(Controller):
	def __init__(self):
		Controller.__init__(self)

		self.pids = {}
		self.pids['u1A'] = dict(Kp=2.0, Ki=0.0, Kd=3.0,
					integrator_limit=(-20, 20),
					pid_limit=(0, 10),
					state=0, reference=0,
					derivator=0, integrator=0,
					gain_factor=1)
		self.pids['u1B'] = dict(Kp=2.0, Ki=0.0, Kd=3.0,
					integrator_limit=(-20, 20),
					pid_limit=(0, 10),
					state=0, reference=0,
					derivator=0, integrator=0,
					gain_factor=self.m*self.g)
		self.pids['u3A'] = dict(Kp=2.0, Ki=0.0, Kd=2.0,
					integrator_limit=(-20, 20),
					pid_limit=None,
					state=0, reference=0,
					derivator=0, integrator=0,
					gain_factor=10*self.Ix)
		self.pids['u3B'] = dict(Kp=2.0, Ki=0.0, Kd=4.0,
					integrator_limit=(-20, 20),
					pid_limit=(-np.pi, np.pi),
					state=0, reference=0,
					derivator=0, integrator=0,
					gain_factor=10)
		self.pids['u3C'] = dict(Kp=4.0, Ki=0.0, Kd=2.0,
					integrator_limit=(-20, 20),
					pid_limit=(-np.pi/4, np.pi/4),
					state=0, reference=0,
					derivator=0, integrator=0,
					gain_factor=1/self.g)
		self.pids['u3D'] = dict(Kp=4.0, Ki=0.0, Kd=2.0,
					integrator_limit=(-20, 20),
					pid_limit=None,
					state=0, reference=0,
					derivator=0, integrator=0,
					gain_factor=1)
		self.pids['u2A'] = dict(Kp=2.0, Ki=0.0, Kd=2.0,
					integrator_limit=(-20, 20),
					pid_limit=None,
					state=0, reference=0,
					derivator=0, integrator=0,
					gain_factor=10*self.Iy)
		self.pids['u2B'] = dict(Kp=2.0, Ki=0.0, Kd=4.0,
					integrator_limit=(-20, 20),
					pid_limit=(-np.pi, np.pi),
					state=0, reference=0,
					derivator=0, integrator=0,
					gain_factor=10)
		self.pids['u2C'] = dict(Kp=4.0, Ki=0.0, Kd=2.0,
					integrator_limit=(-20, 20),
					pid_limit=(-np.pi/4, np.pi/4),
					state=0, reference=0,
					derivator=0, integrator=0,
					gain_factor=1/self.g)
		self.pids['u2D'] = dict(Kp=4.0, Ki=0.0, Kd=2.0,
					integrator_limit=(-20, 20),
					pid_limit=None,
					state=0, reference=0,
					derivator=0, integrator=0,
					gain_factor=1)
		self.pids['u4A'] = dict(Kp=2.0, Ki=0.5, Kd=2.5,
					integrator_limit=(-20, 20),
					pid_limit=None,
					state=0, reference=0,
					derivator=0, integrator=0,
					gain_factor=10*self.Iz)
		self.pids['u4B'] = dict(Kp=2.0, Ki=0.0, Kd=2.5,
					integrator_limit=(-20, 20),
					pid_limit=None,
					state=0, reference=0,
					derivator=0, integrator=0,
					gain_factor=1)

		for pid in self.pids:
			self.pids[pid]['Kp'] *= self.pids[pid]['gain_factor']
			self.pids[pid]['Ki'] *= self.pids[pid]['gain_factor']
			self.pids[pid]['Kd'] *= self.pids[pid]['gain_factor']

	def __call__(self, y, r):
		self.pids['u1A']['state'] = y[5]
		self.pids['u1B']['state'] = y[4]
		self.pids['u3A']['state'] = y[9]
		self.pids['u3B']['state'] = y[8]
		self.pids['u3C']['state'] = y[1]
		self.pids['u3D']['state'] = y[0]
		self.pids['u2A']['state'] = y[7]
		self.pids['u2B']['state'] = y[6]
		self.pids['u2C']['state'] = y[3]
		self.pids['u2D']['state'] = y[2]
		self.pids['u4A']['state'] = y[11]
		self.pids['u4B']['state'] = y[10]

		self.pids['u1B']['reference'] = r[2][0]
		self.pids['u2D']['reference'] = r[1][0]
		self.pids['u3D']['reference'] = r[0][0]
		self.pids['u4B']['reference'] = r[3][0]

		self.pids['u1A']['reference'] = 	self.get_pid(self.pids['u1B'])
		self.u[0] = self.get_pid(self.pids['u1A'])

		self.pids['u2C']['reference'] = 	self.get_pid(self.pids['u2D'])
		self.pids['u2B']['reference'] = 	self.get_pid(self.pids['u2C']) * -1.0
		self.pids['u2A']['reference'] = 	self.get_pid(self.pids['u2B'])
		self.u[1] =				self.get_pid(self.pids['u2A'])
		self.pids['u3C']['reference'] = 	self.get_pid(self.pids['u3D'])
		self.pids['u3B']['reference'] = 	self.get_pid(self.pids['u3C'])
		self.pids['u3A']['reference'] =		self.get_pid(self.pids['u3B'])
		self.u[2] =				self.get_pid(self.pids['u3A'])

		self.pids['u4A']['reference'] = 	self.get_pid(self.pids['u4B'])
		self.u[3] = self.get_pid(self.pids['u4A'])

		ur = [	self.pids['u1A']['reference'],
			self.pids['u2A']['reference'],
			self.pids['u3A']['reference'],
			self.pids['u4A']['reference']	]
		return self.u, ur

	def get_pid(self, pid):
		error = pid['reference'] - pid['state']
		P = pid['Kp'] *  error
		D = pid['Kd'] * (error - pid['derivator'])
		pid['derivator'] = error

		pid['integrator'] += error
		if   pid['integrator'] > pid['integrator_limit'][1]: pid['integrator'] = pid['integrator_limit'][1]
		elif pid['integrator'] < pid['integrator_limit'][0]: pid['integrator'] = pid['integrator_limit'][0]
		I = pid['integrator'] * pid['Ki']

		PID = P + I + D

		if pid['pid_limit']:
			if   PID < pid['pid_limit'][0]: PID = pid['pid_limit'][0]
			elif PID > pid['pid_limit'][1]: PID = pid['pid_limit'][1]

		return PID

