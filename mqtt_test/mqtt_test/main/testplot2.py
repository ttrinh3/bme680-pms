import csv
from matplotlib.animation import FuncAnimation
import matplotlib.pyplot as plt #Enter 'pip install matplotlib' in terminal
from itertools import count
import pandas as pd

def Node1(i):
    data = pd.read_csv('document2.csv', header= 0, encoding = 'unicode_escape')

    PM1 = data['PM1']
    PM2 = data['PM2']
    PM3 = data['PM3']
    Temp = data['Temp']
    Humid = data['Humid']
    Press = data['Press']
    Resist = data['Resist']

    ax1.cla()
    ax1.plot(PM1.index, PM1, label='PM1.0')
    ax1.plot(PM2.index, PM2, label='PM2.5')
    ax1.plot(PM3.index, PM3, label='PM10')
    ax1.legend(loc='upper left')

    ax2.cla()
    ax2.plot(Temp.index, Temp, label='Temperature')
    ax2.legend(loc='upper left')

    ax3.cla()
    ax3.plot(Humid.index, Humid, label='Humidity')
    ax3.legend(loc='upper left')

    ax4.cla()
    ax4.plot(Press.index, Press, label='Pressure')
    ax4.legend(loc='upper left')

    ax5.cla()
    ax5.plot(Resist.index, Resist, label='Resistance')
    ax5.legend(loc='upper left')

#Node1Figs, (PMS1, Temper1, Humidity1, Pressure1, Resistance1) = plt.subplots(nrows=5, ncols=1)

PMS = plt.figure("Node2: PMS Data")
ax1 = plt.subplot()

Temper = plt.figure("Node2: Temperature")
ax2 = plt.subplot()

Humidity = plt.figure("Node2: Humidity")
ax3 = plt.subplot()

Pressure = plt.figure("Node2: Pressure")
ax4 = plt.subplot()

Resistance = plt.figure("Node2: Gas Resistance")
ax5 = plt.subplot()

ani1 = FuncAnimation(PMS, Node1, interval=1000)
ani2 = FuncAnimation(Temper, Node1, interval=1000)
ani3 = FuncAnimation(Humidity, Node1, interval=1000)
ani4 = FuncAnimation(Pressure, Node1, interval=1000)
ani5 = FuncAnimation(Resistance, Node1, interval=1000)

plt.tight_layout()
plt.show() #If graph does not show run 'pip install pyqt5'