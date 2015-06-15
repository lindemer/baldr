# baldr
An open source flight simulator for aerial drones written entirely in Python. This is currently a Linux-only project. baldr is an extension for the MORSE robotics simulator: https://www.openrobots.org/wiki/morse/.

Here's a short video demonstration of v1.0: https://www.youtube.com/watch?v=0st4ma6SF8Y&feature=youtu.be.

## Installation Instructions

1. Install the required Python libraries if you don't already have them (the MORSE robotics simulator requires Python3):
	`$ sudo apt-get install python3 python3-scipy python3-numpy python3-matplotlib`

2. Install MORSE with the command:
	`$ sudo apt-get install morse-simulator`.
	If you don't already have Blender, this should install it automatically. Use the default settings (ie. make the default folder for simulation files your Home folder).

3. You need to create a simulation called "baldr" in MORSE. Do this with the command:
	`$ morse create baldr`.
	This will create a folder in your Home directory called ~/baldr with some MORSE simulation files in it, and it will also add a line to the ~/.morse/config file to identify the name "baldr" as a MORSE simulation.

4. Clone this repository with `$ git clone https://github.com/motet/baldr.git` and replace all the contents of the new ~/baldr folder with the contents of the repository you just downloaded.

5. To run the MORSE component of the simulator, use the command:
	`$ morse run baldr`

6. To run the GUI component of the simulator, use the command:
	`$ python3 ~/baldr/scripts/main.py`
