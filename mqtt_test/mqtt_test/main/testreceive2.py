import random
import csv

from paho.mqtt import client as mqtt_client


broker = 'mqtt.eclipseprojects.io'
port = 1883
topic = "topic/my/timothy/2"
# generate client ID with pub prefix randomly
client_id = f'python-mqtt-{random.randint(0, 100)}'
username = 'emqx'
password = 'public'

Node = 0
PM1 = 0
PM2 = 0
PM3 = 0
Temp = 0
Humid = 0
Press = 0
Resist = 0
#Time = 0
fieldnames = ["Node", "PM1", "PM2", "PM3", "Temp", "Humid", "Press", "Resist"]

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


def subscribe(client: mqtt_client):
    def on_message(client, userdata, msg):
        print(f"Received {msg.payload.decode()} from {msg.topic} topic")
        with open('document2.csv','a') as csv_file:
            csv_writer = csv.DictWriter(csv_file, fieldnames=fieldnames)
            PM1 = str(msg.payload.decode()).split(',')[0]
            PM2 = str(msg.payload.decode()).split(',')[1]
            PM3 = str(msg.payload.decode()).split(',')[2]
            Temp = str(msg.payload.decode()).split(',')[3]
            Humid = str(msg.payload.decode()).split(',')[4]
            Press = str(msg.payload.decode()).split(',')[5]
            Resist = str(msg.payload.decode()).split(',')[6]
            #Time = str(msg.payload.decode()).split(',')[7]
            info = {
                "Node": Node,
                "PM1": PM1,
                "PM2": PM2,
                "PM3": PM3,
                "Temp": Temp,
                "Humid": Humid,
                "Press": Press,
                "Resist": Resist
                #"Time": Time
            }

            csv_writer.writerow(info)
            print(Node, PM1, PM2, PM3, Temp, Humid, Press, Resist)
            

    client.subscribe(topic)
    client.on_message = on_message


def run():
    client = connect_mqtt()
    subscribe(client)
    client.loop_forever()

if __name__ == '__main__':
    with open('document2.csv', 'w') as csv_file:
        csv_writer = csv.DictWriter(csv_file, fieldnames=fieldnames)
        csv_writer.writeheader()
    run()