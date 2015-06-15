#! /usr/bin/env python3
'''
	Developed by Samuel Tanner Lindemer 2015
	Le Laboaratoire des Signaux et Systemes
	Centrale-Supelec Universite, Gif-sur-Yvette, France
	samuel.lindemer@lss.supelec.fr
'''

from multiprocessing import Process, Queue

def component_handler(request, progress_q, simulation_q):
	import time
	if request['component'] == 'draw':
		import draw
		editor = draw.WindowManager(*request['draw']['args'], **request['draw']['kwargs'])

	elif request['component'] == 'simulator':
		# simulation settings received, initialize
		import simulators
		if request['simulator']['name'] == 'matplotlib': 
			sim = simulators.MatplotlibAnalysis(progress_q, **request['simulator']['kwargs'])
			if request['controller']['name'] == 'cascade':	sim.set_controller('cascade tracking')
			else:						sim.set_controller(request['controller']['name'])
		elif request['simulator']['name'] == 'MORSE (tracking)': 
			sim = simulators.MorseTracking(progress_q, **request['simulator']['kwargs'])
			if request['controller']['name'] == 'cascade':	sim.set_controller('cascade tracking')
			else:						sim.set_controller(request['controller']['name'])
		elif request['simulator']['name'] == 'MORSE (interactive)': 
			sim = simulators.MorseInteractive(progress_q, **request['simulator']['kwargs'])
			if request['controller']['name'] == 'cascade':	sim.set_controller('cascade interactive')
			else:						sim.set_controller(request['controller']['name'])
		sim.set_model(request['model']['name'])
		if request['simulator']['name'] != 'MORSE (interactive)': sim.set_trajectory(request['trajectory']['name'], **request['trajectory']['kwargs'])
		sim.set_integrator(request['integrator']['name'], **request['integrator']['kwargs'])
		if request['simulator']['name'] != 'MORSE (interactive)': sim.process_data()
		while True:
			time.sleep(0.1)		# save system resources
			try: command = simulation_q.get(block=False)
			except: pass
			else: 
				if command == 'start': sim.run()
			
def request_handler(request_q, progress_q, simulation_q):
	import sys
	import time
	while True:
		time.sleep(0.1)			# save system resources
		try: request = request_q.get(block=False)
		except: pass
		else:
			if request == 'terminate':
				# kill the simulation process
				simulation_process.terminate()
			elif request == 'exit':
				# request_handler is a non-daemonic process because a daemonic process cannot spawn a new process...
				# therefore, we must have an exit request on the queue to manually kill the process when the gui
				# window closes
				sys.exit(1)
			else: # simulation description received, spawn new process
				simulation_process = Process(target=component_handler, args=(request, progress_q, simulation_q))
				simulation_process.daemon = True
				simulation_process.start()

if __name__ == '__main__':
	print('\n\
Baldr Quadrotor Simulator\n\
Developed by Samuel Tanner Lindemer 2015\n\
Le Laboratoire des Signaux et Systemes\n\
Centrale-Supelec Universite, Gif-sur-Yvette, France\n\
samuel.lindemer@lss.supelec.fr', end='\n\n')

	# this queue transfers data from the main gui process to the request_handler process (terminate, exit, initialize)
	request_q = Queue()
	# this queue transfers data from the simulation_handler process to the main gui process (progressbar data)
	progress_q = Queue()
	# this queue transfers data from the main gui process to the simulation_handler process (start, open, save)
	simulation_q = Queue()

	handler_process = Process(target=request_handler, args=(request_q, progress_q, simulation_q))
	handler_process.daemon = False
	handler_process.start()

	import gui
	gui_window = gui.Baldr(request_q, progress_q, simulation_q)
	gui_window.tk_mainloop()

