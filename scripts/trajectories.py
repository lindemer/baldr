'''
	Developed by Samuel Tanner Lindemer 2015
	Le Laboaratoire des Signaux et Systemes
	Centrale-Supelec Universite, Gif-sur-Yvette, France
	samuel.lindemer@lss.supelec.fr
'''

import numpy as np

class Tangential():
	def __init__(self):
		t0 = 0
		t1 = 10
		self.st = 1		# ref traj stiffness
		self.lo = 0		# min speed
		self.hi = 10		# max speed
		self.tR = t0 + 2 * (t1 - t0) / 10
		self.aR  = (self.hi - self.lo) / 2

	def __call__(self, t):
		phi =  np.tanh(self.st * (t - self.tR))
		d0  =  self.aR * (1 + phi) + self.lo
		d1  =  self.aR * self.st * (1 - phi**2)
		d2  =  2 * self.aR * self.st**2 * phi * (1 - phi**2)
		d3  = -2 * self.aR * self.st**3 * (1 - phi**2) * (1 - 3 * phi**2)
		d4  =  8 * self.aR * self.st**4 * phi * (3 * phi**4 - 5 * phi**2 + 2)
		d5  = -8 * self.aR * self.st**5 * (15 * phi**6 - 30 * phi**4 + 17 * phi**2 - 2)

		r = np.array([	[d0, d1, d2, d3, d4],
				[d0, d1, d2, d3, d4],
				[d0, d1, d2, d3, d4],
				[d0, d1, d2, d3, d4]	])

		return r

class Spiral():
	def __call__(self, t):
		x     = t *  np.cos(t)
		dx    = t * -np.sin(t) + np.cos(t)
		d2x   = t * -np.cos(t) - np.sin(t) * 2
		d3x   = t *  np.sin(t) - np.cos(t) * 3
		d4x   = t *  np.cos(t) + np.sin(t) * 4
		y     = t *  np.sin(t)
		dy    = t *  np.cos(t) + np.sin(t)
		d2y   = t * -np.sin(t) + np.cos(t) * 2
		d3y   = t * -np.cos(t) - np.sin(t) * 3
		d4y   = t *  np.sin(t) - np.cos(t) * 4
		z     = t
		dz    = 1
		d2z   = 0
		d3z   = 0
		d4z   = 0
		psi   = t * 0.5 * np.pi
		dpsi  = 0.5 * np.pi
		d2psi = 0
		d3psi = 0
		d4psi = 0

		r = np.array([	[x,    dx,    d2x,   d3x,   d4x  ],
				[y,    dy,    d2y,   d3y,   d4y  ],
				[z,    dz,    d2z,   d3z,   d4z  ],
				[psi,  dpsi,  d2psi, d3psi, d4psi]   ])

		return r

