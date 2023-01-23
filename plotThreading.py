#!/usr/bin/env python

from threading import Thread
import serial
import time
import collections
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import struct
import copy
import pandas as pd

class serialPlot:
    def __init__(self, serialPort='/dev/ttyACM0', serialBaud=115200, plotLength=100, dataNumBytes=4, numPlots=1):
        self.port = serialPort
        self.baud = serialBaud
        self.plotMaxLength = plotLength
        self.dataNumBytes = dataNumBytes
        self.numPlots = numPlots
        self.rawData = bytearray(numPlots * dataNumBytes)
        self.dataType = None
        if dataNumBytes == 2:
            self.dataType = 'h'     # 2 byte integer
        elif dataNumBytes == 4:
            self.dataType = 'f'     # 4 byte float
        elif dataNumBytes == 8:
            self.dataType = 'd'     # 8 byte due double float           
        self.data = []
        self.plotdata = []
        for i in range(numPlots):   # give an array for each type of data and store them in a list
            self.data.append(collections.deque([0] * plotLength, maxlen=plotLength))
        for i in range(numPlots +1):    
            self.plotdata.append(collections.deque([0] * plotLength, maxlen=plotLength))
        self.isRun = True
        self.isReceiving = False
        self.thread = None
        self.plotTimer = 0
        self.previousTimer = 0
        self.csvData = []
        self.MINUTE_VOLUME_WINDOW_SIZE = 200 #averaging over a half second with 5mS sample rate
        self.flow_average_data = []
        self.flow_moving_average_sum = 0
        self.flowRate = 0

        print('Trying to connect to: ' + str(serialPort) + ' at ' + str(serialBaud) + ' BAUD.')
        try:
            self.serialConnection = serial.Serial(serialPort, serialBaud, timeout=4)
            print('Connected to ' + str(serialPort) + ' at ' + str(serialBaud) + ' BAUD.')
        except:
            print("Failed to connect with " + str(serialPort) + ' at ' + str(serialBaud) + ' BAUD.')


    def readSerialStart(self):
        if self.thread == None:
            self.thread = Thread(target=self.backgroundThread)
            self.thread.start()
            # Block till we start receiving values
            while self.isReceiving != True:
                time.sleep(0.1)


    def getSerialData(self):
        currentTimer = time.perf_counter()
        #currentTimer = time.clock()
        self.plotTimer = int((currentTimer - self.previousTimer) * 1000)     # the first reading will be erroneous
        self.previousTimer = currentTimer
        privateData = copy.deepcopy(self.rawData[:])    # so that the 3 values in our plots will be synchronized to the same sample time
        for i in range(self.numPlots): #time(micros),flow(lpm),temp(C),pressure(psi)
            data = privateData[(i*self.dataNumBytes):(self.dataNumBytes + i*self.dataNumBytes)]
            value,  = struct.unpack(self.dataType, data)
            self.data[i].append(value)    # we get the latest data point and append it to our array
            self.plotdata[i].append(value)
        rawFlowValue=self.data[1][-1]
        self.flowRate=self.calculateFlowVolume(rawFlowValue)
        self.plotdata[self.numPlots].append(self.flowRate)
        
        self.csvData.append([self.data[0][-1], self.data[1][-1], self.data[2][-1], self.data[3][-1], self.plotdata[self.numPlots][-1]])

        
    def calculateFlowVolume(self,currentFlowValue):
        #flow sensor scale and offset are applied at arduino microprosessor
        ##flowSensorValSLM = (flowSensorVal - flowSensorOffset)/((float)(flowSensorScale));
        
        #flow sensor reports in lpm but at undesired temp and press, we want STDP0
        adj_flow = (273.15/293.15)*(101300.0/101325.0)*currentFlowValue;
        average_flow =self.processMovingAverage(adj_flow)
        
        return average_flow
    
    def processMovingAverage(self, new_value):
        self.flow_average_data.append(new_value)
        self.flow_moving_average_sum += new_value
        if len(self.flow_average_data) >  self.MINUTE_VOLUME_WINDOW_SIZE:
            self.flow_moving_average_sum -= self.flow_average_data.pop(0)
        return float(self.flow_moving_average_sum) / len(self.flow_average_data)
    
    def backgroundThread(self):    # retrieve data
        time.sleep(0.005)  # give some buffer time for retrieving data
        self.serialConnection.reset_input_buffer()
        while (self.isRun):
            self.serialConnection.readinto(self.rawData)
            self.isReceiving = True
            self.getSerialData()
            
            #print(self.rawData)
    def animate(self, frame, lines, lineValueText, lineLabel, timeText):
        for i in range(self.numPlots): #time(micros),flow(lpm),temp(C),pressure(psi)
            lines[i].set_data(range(self.plotMaxLength), self.plotdata[i])#this line is what feeds the data to the plots
            #lineValueText[i].set_text('[' + lineLabel[i] + '] = ' + str(value))
        lines[self.numPlots].set_data(range(self.plotMaxLength), self.plotdata[self.numPlots])
        lineValueText[self.numPlots].set_text('[' + lineLabel[self.numPlots] + '] = ' + str(self.flowRate))        
            
    def close(self):
        self.isRun = False
        self.thread.join()
        self.serialConnection.close()
        print('Disconnected...')
        df = pd.DataFrame(self.csvData)
        df.to_csv('C:/Users/Justin/Documents/PythonCode/data.csv')


def main():
    #portName = '/dev/ttyACM0'
    portName = 'COM5'
    baudRate = 115200
    maxPlotLength = 500     # number of points in x-axis of real time plot
    dataNumBytes = 4        # number of bytes of 1 data point
    numPlots = 4            # number of plots in 1 graph
    s = serialPlot(portName, baudRate, maxPlotLength, dataNumBytes, numPlots)   # initializes all required variables
    s.readSerialStart()                                               # starts background thread

    # plotting starts below
    pltInterval = 25 # Period at which the plot animation updates [ms]
    xmin = 0
    xmax = maxPlotLength
    ymin =  0
    ymax = 6
    fig = plt.figure(figsize=(10, 8))
    ax = plt.axes(xlim=(xmin, xmax), ylim=(float(ymin - (ymax - ymin) / 10), float(ymax + (ymax - ymin) / 10)))
    ax.set_title('Flow Volume Testing')
    ax.set_xlabel("Time")
    ax.set_ylabel("Sensor Output")

    lineLabel = ['time (micros)', 'raw flow',  'Temp (C)', 'Pressure (psi)','average Flow rate (lpm)']
    style = ['k-', 'b-', 'r-', 'g-','c-']  # linestyles for the different plots
    timeText = ax.text(0.70, 0.95, '', transform=ax.transAxes)
    lines = []
    lineValueText = []
    for i in range(numPlots+1):
        lines.append(ax.plot([], [], style[i], label=lineLabel[i])[0])
        lineValueText.append(ax.text(0.40, 0.90-i*0.05, '', transform=ax.transAxes))
    anim = animation.FuncAnimation(fig, s.animate, fargs=(lines, lineValueText, lineLabel, timeText), interval=pltInterval)    # fargs has to be a tuple

    plt.legend(loc="upper left")
    plt.show()

    s.close()

if __name__ == '__main__':
    main()
