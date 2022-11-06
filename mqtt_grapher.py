import random
import csv

from paho.mqtt import client as mqtt_client

# import pylab as plt
import numpy as np
import matplotlib.pyplot as plt


broker = 'mqtt.eclipseprojects.io'
port = 1883
topic = "topic/my/timothy/1"
# generate client ID with pub prefix randomly
client_id = f'python-mqtt-{random.randint(0, 100)}'
username = 'emqx'
password = 'public'

#Initialize a an array for the values
#each value type is a row so 9
#let's just start with a buffer of 10
node1_data = np.zeros((7, 100))
node2_data = np.zeros((7, 100))
#init the interactive plot



  #node, pm1, pm2, pm3, temp, humidity, pressure, resistance, time 
plt.ion()
fig = plt.figure(constrained_layout=True)

#init subplot
node1_pm1_0 = fig.add_subplot(7,2,1)
node1_pm2_5 = fig.add_subplot(7,2,3)
node1_pm10 = fig.add_subplot(7,2,5)
node1_temp = fig.add_subplot(7,2,7)
node1_humidity = fig.add_subplot(7,2,9)
node1_pressure = fig.add_subplot(7,2,11)
node1_resistance = fig.add_subplot(7,2,13)
node2_pm1_0 = fig.add_subplot(7,2,2)
node2_pm2_5 = fig.add_subplot(7,2,4)
node2_pm10 = fig.add_subplot(7,2,6)
node2_temp = fig.add_subplot(7,2,8)
node2_humidity = fig.add_subplot(7,2,10)
node2_pressure = fig.add_subplot(7,2,12)
node2_resistance = fig.add_subplot(7,2,14)
#create a line for subplot
line1, = node1_pm1_0.plot(node1_data[0][:])
line2, = node1_pm2_5.plot(node1_data[1][:])
line3, = node1_pm10.plot(node1_data[2][:])
line4, = node1_temp.plot(node1_data[3][:])
line5, = node1_humidity.plot(node1_data[4][:])
line6, = node1_pressure.plot(node1_data[5][:])
line7, = node1_resistance.plot(node1_data[6][:])
line8, = node2_pm1_0.plot(node2_data[0][:])
line9, = node2_pm2_5.plot(node2_data[1][:])
line10, = node2_pm10.plot(node2_data[2][:])
line11, = node2_temp.plot(node2_data[3][:])
line12, = node2_humidity.plot(node2_data[4][:])
line13, = node2_pressure.plot(node2_data[5][:])
line14, = node2_resistance.plot(node2_data[6][:])

#set its title
# node1_pm1_0.set_title("Node 1 PM1.0")
# node1_pm2_5.set_title("Node 1 PM2.5")
# node1_pm10.set_title("Node 1 PM10")
# node1_temp.set_title("Node 1 temp")
# node1_humidity.set_title("Node 1 Humidity")
# node1_pressure.set_title("Node 1 Pressure") 
# node1_resistance.set_title("Node 1 Resistance")
# node2_pm1_0.set_title("Node 2 PM1.0")
# node2_pm2_5.set_title("Node 2 PM2.5")
# node2_pm10.set_title("Node 2 PM10")
# node2_temp.set_title("Node 2 temp")
# node2_humidity.set_title("Node 2 Humidity")
# node2_pressure.set_title("Node 2 Pressure") 
# node2_resistance.set_title("Node 2 Resistance")




def connect_mqtt() -> mqtt_client:
    def on_connect(client, userdata, flags, rc):
        if rc == 0:
            print("Connected to MQTT Broker!")
        else:
            print("Failed to connect, return code %d\n", rc)

    client = mqtt_client.Client(client_id)

    client.on_connect = on_connect
    client.connect(broker, port)
    return client

#node, pm1, pm2, pm3, temp, humidity, pressure, resistance, time 
#step put each value in its own row, the number of columns is the buffer size

def subscribe(client: mqtt_client):
    
    def on_message(client, userdata, msg):
        # print(f"Received {msg.payload.decode()} from {msg.topic} topic")
        with open('document.csv','a',newline='') as fd:
            spamwriter = csv.writer(fd)
            temp = str(msg.payload.decode()).split(',')
            
            spamwriter.writerow([temp[i] for i in range(len(temp))])
        #gather one line of code
        temp = str(msg.payload.decode()).split(',')
        if (temp[0]=='Node1'):
            global node1_data
            #transfer all data to node1
            node1_data=np.roll(node1_data,1) #push all data to the right

            for i in range(1,8): #-2 because we're ignoring the non sensor values which are first and last
                node1_data[i-1][0] = temp[i]
            # print ("start node1",node1_data,"end node1")
            # print(node1_data[][:])
        else:
            global node2_data
            node2_data=np.roll(node2_data,1)
            for i in range(1,8):
                node2_data[i-1][0] = temp[i]
        # graph here
        global line1
        global line2
        #update the line
        line1.set_ydata(node1_data[0][:])
        line2.set_ydata(node1_data[1][:])
        line3.set_ydata(node1_data[2][:])
        line4.set_ydata(node1_data[3][:])
        line5.set_ydata(node1_data[4][:])
        line6.set_ydata(node1_data[5][:])
        line7.set_ydata(node1_data[6][:])
        line8.set_ydata(node2_data[0][:])
        line9.set_ydata(node2_data[1][:])
        line10.set_ydata(node2_data[2][:])
        line11.set_ydata(node2_data[3][:])
        line12.set_ydata(node2_data[4][:])
        line13.set_ydata(node2_data[5][:])
        line14.set_ydata(node2_data[6][:])
        #update the lims
        node1_pm1_0.set_ylim([np.amin(node1_data[0][:]),np.amax(node1_data[0][:])])
        node1_pm2_5.set_ylim([np.amin(node1_data[1][:]),np.amax(node1_data[1][:])])
        node1_pm10.set_ylim([np.amin(node1_data[2][:]),np.amax(node1_data[2][:])])
        node1_temp.set_ylim([np.amin(node1_data[3][:]),np.amax(node1_data[3][:])])
        node1_humidity.set_ylim([np.amin(node1_data[4][:]),np.amax(node1_data[4][:])])
        node1_pressure.set_ylim([np.amin(node1_data[5][:]),np.amax(node1_data[5][:])])
        node1_resistance.set_ylim([np.amin(node1_data[6][:]),np.amax(node1_data[6][:])])
        node2_pm1_0.set_ylim([np.amin(node2_data[0][:]),np.amax(node2_data[0][:])])
        node2_pm2_5.set_ylim([np.amin(node2_data[1][:]),np.amax(node2_data[1][:])])
        node2_pm10.set_ylim([np.amin(node2_data[2][:]),np.amax(node2_data[2][:])])
        node2_temp.set_ylim([np.amin(node2_data[3][:]),np.amax(node2_data[3][:])])
        node2_humidity.set_ylim([np.amin(node2_data[4][:]),np.amax(node2_data[4][:])])
        node2_pressure.set_ylim([np.amin(node2_data[5][:]),np.amax(node2_data[5][:])])
        node2_resistance.set_ylim([np.amin(node2_data[6][:]),np.amax(node2_data[6][:])])
        fig.canvas.draw()
        fig.canvas.flush_events()

  
        




        # with plt.style.context('ggplot'):
        #     for i in range(node1_data.sha
        # pe[0]):
        #         plt.plot(node1_data[i][:])



        # graph = plt.plot(X,Y)[0]

        # Y = X**2 + np.random.random(X.shape)
        # graph.set_ydata(Y)
    
    client.subscribe(topic)
    client.on_message = on_message


def run():
    client = connect_mqtt()
    subscribe(client)
    client.loop_forever()


if __name__ == '__main__':
    run()