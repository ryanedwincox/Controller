#!/usr/bin/env python

import Tkinter as tk
from Tkinter import*

class TunerGui(tk.Frame):	
	def __init__(self, master=None):
		# Initialize root frame
		tk.Frame.__init__(self, master)  
		
		# reads slider
		self.getSlider()
		self.grid()


	def getSlider(self):
		# create label frame for sliders
		self.sliderFrame = Label(self, text= '')
		
		# create empy list of sliders
		self.sliderFrame.constantsScrolls = [None]*12
		self.sliderFrame.decimalScrolls = [None]*12	
		
		# Create list of motor labels
		sliderLabels = ["x Kp", "x Ki", "x Kd", "y Kp", "y Ki", "y Kd", "z Kp", "z Ki", "z Kd", "yaw Kp", "yaw Ki", "yaw Kd"]

		# create list of slider range
		sliderFrom = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
		sliderTo = [100,100,100,100,100,100,100,100,100,1000,100,1000]

		# Create scroll bars for each PID constant
		for i in range(12):
			label = sliderLabels[i]				
			self.sliderFrame.motorScaleLabel = Label(self.sliderFrame, text=label)
			self.sliderFrame.motorScaleLabel.grid(row=i, column=0, sticky=W)
	
			self.sliderFrame.constantsScrolls[i] = Scale(self.sliderFrame, orient=tk.HORIZONTAL, from_=sliderFrom[i],to=sliderTo[i], length=450)
			self.sliderFrame.constantsScrolls[i].grid(row=i, column=1, sticky=W+E)

			self.sliderFrame.decimalScrolls[i] = Scale(self.sliderFrame, orient=tk.HORIZONTAL, from_=0,to=99, length=450)
			self.sliderFrame.decimalScrolls[i].grid(row=i, column=2, sticky=W+E)
			
		self.sliderFrame.grid(row=0, column=0, columnspan=2, ipadx=10, sticky=W+E)		
		

	def getConstants(self):
		constantsList = []
		for i in range(12):
			intVal = self.sliderFrame.constantsScrolls[i].get()
			decVal = self.sliderFrame.decimalScrolls[i].get()
			combinedVal = str(intVal + decVal/100.0)
			constantsList.append(combinedVal)
		return constantsList


