#!/usr/bin/env python

# general stuff
import numpy as np
import subprocess, os, sys
import dill

# helper class to get transitions from funnel to funnel using interactive mode
class SlugsInterface():
	def __init__(self, name, slugsLink = '/home/cornell/Tools/slugs_ltl_stack/src/slugs'):
		self.name = name # name of map file to synthesize
		self.realizeable = True
		self.enabled = False # is the class operational? did initialize and are realizeable
		self._current_state = '' # string that holds the state
		self._trans_state = ''   # string that holds the new state after transitioning
		self._current_goal = 0 
		self._inputs = {} # names and values of current input (states)
		self._outputs = {} # names and values of current output (action)
		self._Nin = 0   # total number of inputs (self robot + restrictions)
		self._Nout = 0  # number of command bits
		self._Nself = 0 # number of self robot bits
		self._Nsens = 0 # number of sensors bits
		
		# open the process of slugs and keep it open to recieve commands
		self.slugsProcess = subprocess.Popen(slugsLink + ' --interactiveStrategy ' + self.name + '.slugsin', \
											 bufsize=1048000, stderr=subprocess.PIPE, shell=True, preexec_fn=os.setsid, \
											 stdin=subprocess.PIPE, stdout=subprocess.PIPE, \
											 close_fds='posix' in sys.builtin_module_names)
		# wait for synthesis to be done
		stderr, stdout = "",""
		
		# wait till we find out if the file is realizeable or not
		while True: 
			# retrieve outputs
			stdout += self.slugsProcess.stdout.readline().strip()
			stderr += self.slugsProcess.stderr.readline().strip()
			# exit if synthesis is done and ready for execution
			if "error" in stderr.lower():
				self.realizeable = False
				break
			elif "unrealizable" in stderr:
				self.realizeable = False
				break
			elif "oneStepRecovery" in stdout:
				break
		if(self.realizeable):		
			print('ready to rock & roll!')
			self.enabled = True
		else:
			print('check your slugsin file, it is not realizeable ...')

	# get the list of inputs
	def DiscoverInputs(self):
		if(self.enabled):
			self.slugsProcess.stdin.write("XPRINTINPUTS\n")
			self.slugsProcess.stdin.flush()

			stdout, stderr = "", ""
			# get the response
			while not stdout.endswith("\n\n"):
				stdout += self.slugsProcess.stdout.readline()
			
			inputs_state = stdout.replace(">","").strip()
			#print("inputs: {inputs_state}".format(inputs_state=inputs_state))
			list_of_inputs = inputs_state.split('\n')
			self._inputs = dict(zip(list_of_inputs, [0]*len(list_of_inputs)))
			self._Nin = len(list_of_inputs)
			self._Nself = 0
			# find out how many occupy the self robot. the idea is that it is represented
			# by R so all it's propositions are R@something
			for x in self._inputs.keys():
				if('R@' in x):
					self._Nself +=1
			self._Nsens = self._Nin - self._Nself
			
	def DiscoverOutputs(self):
		if(self.enabled):
			self.slugsProcess.stdin.write("XPRINTOUTPUTS\n")
			self.slugsProcess.stdin.flush()

			stdout, stderr = "", ""
			while not stdout.endswith("\n\n"):
			#while not ">" in stdout:
				stdout += self.slugsProcess.stdout.readline()
			
			outputs_state = stdout.replace(">","").strip()
			#print("outputs: {outputs_state}".format(outputs_state=outputs_state))
			list_of_outputs = outputs_state.split('\n')
			self._Nout = len(list_of_outputs)
			self._outputs = dict(zip(list_of_outputs, [0]*self._Nout))
			
	# get the list of inputs, returns self_robot, restrictions
	def GetCurrentInputs(self):
		if(self.enabled):
			return self._current_state[0:self._Nself], self._current_state[self._Nself:]
		else:
			return '', ''

	
	# gets current position, if everything is as expected in start up
	def GetInitialPos(self):
		if(self.enabled):
			self.slugsProcess.stdin.write("XGETINIT\n")
			self.slugsProcess.stdin.flush()

			stdout, stderr = "", ""
			while not "," in stdout:
				stdout += self.slugsProcess.stdout.readline()
			
			init_state = stdout.replace(">","").strip()
			#print("Init state: {init_state}".format(init_state=init_state))
			self._current_state = init_state.partition(",")[0]
			# save it also as a list of int
			self._current_state_n = [int(x) for x in list(self._current_state)]
			if init_state.partition(",")[2]:
				self._current_goal = init_state.partition(",")[2]
			return self._current_state
		else:
			return ''
	
	# sets current position, if you need to reset for some reason
	def SetCurrentPos(self, state, sense, action):
		if(self.enabled):
			state_str = format(state, '0%db' %(self._Nself) )
			
			state_str = state_str[::-1]
			# convert dict of restrictions into a string
			sensed_inputs = ""
			for prop in range(self._Nsens):
				if prop in sense:
					sensed_inputs += "1" if (sense[prop] is True) else "0"
				else:
					sensed_inputs += "."
					
			state_str = state_str + sensed_inputs # restrictions
			state_str = state_str + format(action, '04b')[::-1] # actions
			
			self.slugsProcess.stdin.write("SETPOS\n" + state_str.replace("1","1\n").replace("0","0\n"))
			self.slugsProcess.stdin.flush()

			self._current_state = state_str
			self._current_state_n = [int(x) for x in list(self._current_state)]
			return self._current_state
		else:
			return ''

	# gets current position, if everything is as expected in start up
	def SetGoal(self, goal=0):
		if(self.enabled):
			self.slugsProcess.stdin.write("XMAKEGOAL\n" + str(goal))
			self.slugsProcess.stdin.flush()
			
			self._current_goal = str(goal)

		
	# in: the sensed robot location & sensors of the available funnels
	def FindNextStep(self, robot_sensed_state, funnel_occupancy_dict):
		if(self.enabled):
			#import pdb; pdb.set_trace()
			# do stuff to create the reversed binary string slugs expects
			sensed_state = format(robot_sensed_state, '0%db' %(self._Nself) )
			sensed_state = sensed_state[::-1]
			
			# convert dict of restrictions into a string
			sensed_inputs = ""
			for prop in range(self._Nsens):
				if prop in funnel_occupancy_dict:
					sensed_inputs += "1" if (funnel_occupancy_dict[prop] is True) else "0"
				else:
					sensed_inputs += "."
				
			sensed_state = sensed_state + sensed_inputs 
			self.slugsProcess.stdin.write("XMAKETRANS\n" + sensed_state)
			self.slugsProcess.stdin.flush()

			# retrieve the new state
			stdout, stderr = "", ""
			while not ("," in stdout or "error" in stdout.lower()):
				stdout += self.slugsProcess.stdout.readline()
			
			trans = stdout.replace(">","").strip()
			#print("transition to state: {trans}".format(trans=trans))

			self._trans_state = trans.partition(",")[0]
			self._trans_state_n = [int(x) for x in list(self._trans_state)]
			
			next_state_str = self._trans_state[0:self._Nself]
			next_state     = int(next_state_str[::-1], 2)
			next_cmd_str   = self._trans_state[-self._Nout:]
			next_cmd       = int(next_cmd_str[::-1], 2)
			
			return next_state, next_cmd
		else:
			return ''
			
	# tell the class the you've actually made the transition
	def MoveNextStep(self):
		if(self.enabled):
			self._current_state = self._trans_state
			self._current_state_n = self._trans_state_n

	# gets the actual numbers from the strings, so it could be used easier from outside
	def GetNumericState(self):
		if(self.enabled):
			#import pdb; pdb.set_trace()
			next_state_str = self._current_state[0:self._Nself]
			next_state     = int(next_state_str[::-1], 2)
			next_cmd_str   = self._current_state[-self._Nout:]
			next_cmd       = int(next_cmd_str[::-1], 2)

			return next_state, next_cmd
		else:
			return -1, -1
	
	def Shutdown(self):
		if(self.enabled):
			self.slugsProcess.terminate()
			self.slugsProcess.wait()
			self.enabled = False
	
if __name__ == "__main__":
	fname = 'smalllab'
	import networkx as nx
	G = nx.read_gpickle(fname + '.pickle')
	
	with open(fname + '.label2bit', 'rb') as dbfile:
		map_label_2_bit = dill.load(dbfile)
		map_bit_2_label = dict((v, k) for k, v in map_label_2_bit.items())
	
	SI = SlugsInterface(fname + '_r0')
	SI.DiscoverInputs()
	SI.DiscoverOutputs()
	SI.GetInitialPos()
	cur_state = 0
	sensors = dict(zip(range(SI._Nsens), [False]*SI._Nsens))
	#import pdb; pdb.set_trace()
	
	for i in range(20):
		#if(i==15):
		#	import pdb; pdb.set_trace()
		cur_state, cur_cmd = SI.GetNumericState()
		cur_state_label = map_bit_2_label[cur_state]
		next_state = -1
		if(cur_cmd==9):
			# stuck, stay in place
			SI.SetGoal(goal=0)
			next_state = cur_state_label
		else:
			for key, val in G[cur_state_label].items():
				if( val['motion'] == cur_cmd):
					next_state = key
					break
		#import pdb; pdb.set_trace()
		SI.FindNextStep(map_label_2_bit[next_state], sensors)
		print('%d) Were in %s (%d), took action %d to get to %s(%d)' \
			  %(i+1, cur_state_label, cur_state, cur_cmd, next_state, map_label_2_bit[next_state]))
		SI.MoveNextStep() # this simulates the fact that we eventually got to where we need
	SI.Shutdown()
	#import pdb; pdb.set_trace()
	
	
	
	
	
	
	
	