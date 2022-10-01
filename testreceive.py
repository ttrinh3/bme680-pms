import random
import csv

from paho.mqtt import client as mqtt_client


broker = 'mqtt.eclipseprojects.io'
port = 1883
topic = "topic/my/timothy/1"
# generate client ID with pub prefix randomly
client_id = f'python-mqtt-{random.randint(0, 100)}'
username = 'emqx'
password = 'public'

PM1 = 0
PM2 = 0
PM3 = 0
Temp_680 = 0
Humid_680 = 0
Press_680 = 0
Resist_680 = 0
Temp_280 = 0
Humid_280 = 0
Press_280 = 0
Time = 0
fieldnames = ["PM1", "PM2", "PM3", "680 Temp", "680 Humid", "680 Press", "280 Temp", "280 Humid", "280 Press", "Resist", "Time"]

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
        node_num = str(msg.payload.decode()).split(',')[0]
        if node_num == "Node1":
            with open('New1.csv','a') as csv_file:
                csv_writer = csv.DictWriter(csv_file, fieldnames=fieldnames)
                PM1 = str(msg.payload.decode()).split(',')[1]
                PM2 = str(msg.payload.decode()).split(',')[2]
                PM3 = str(msg.payload.decode()).split(',')[3]
                Temp_680 = str(msg.payload.decode()).split(',')[4]
                Humid_680 = str(msg.payload.decode()).split(',')[5]
                Press_680 = str(msg.payload.decode()).split(',')[6]
                Resist_680 = str(msg.payload.decode()).split(',')[7]
                Temp_280 = str(msg.payload.decode()).split(',')[8]
                Humid_280 = str(msg.payload.decode()).split(',')[9]
                Press_280 = str(msg.payload.decode()).split(',')[10]
                Time = str(msg.payload.decode()).split(',')[11]
                info1 = {
                    "PM1": PM1,
                    "PM2": PM2,
                    "PM3": PM3,
                    "680 Temp": Temp_680,
                    "680 Humid": Humid_680,
                    "680 Press": Press_680,
                    "680 Resist": Resist_680,
                    "280 Temp": Temp_280,
                    "280 Humid": Humid_280,
                    "280 Press": Press_280,
                    "Time": Time
                }

                csv_writer.writerow(info1)
                print(PM1, PM2, PM3, Temp_680, Humid_680, Press_680, Resist_680, Temp_280, Humid_280, Press_280, Time)

        elif node_num == "Node2":
            with open('New2.csv', 'a') as csv_file:
                csv_writer = csv.DictWriter(csv_file, fieldnames=fieldnames)
                PM1 = str(msg.payload.decode()).split(',')[1]
                PM2 = str(msg.payload.decode()).split(',')[2]
                PM3 = str(msg.payload.decode()).split(',')[3]
                Temp_680 = str(msg.payload.decode()).split(',')[4]
                Humid_680 = str(msg.payload.decode()).split(',')[5]
                Press_680 = str(msg.payload.decode()).split(',')[6]
                Resist_680 = str(msg.payload.decode()).split(',')[7]
                Temp_280 = str(msg.payload.decode()).split(',')[8]
                Humid_280 = str(msg.payload.decode()).split(',')[9]
                Press_280 = str(msg.payload.decode()).split(',')[10]
                Time = str(msg.payload.decode()).split(',')[11]
                info2 = {
                    "PM1": PM1,
                    "PM2": PM2,
                    "PM3": PM3,
                    "680 Temp": Temp_680,
                    "680 Humid": Humid_680,
                    "680 Press": Press_680,
                    "680 Resist": Resist_680,
                    "280 Temp": Temp_280,
                    "280 Humid": Humid_280,
                    "280 Press": Press_280,
                    "Time": Time
                }

                csv_writer.writerow(info2)
                print(PM1, PM2, PM3, Temp_680, Humid_680, Press_680, Resist_680, Temp_280, Humid_280, Press_280, Time)
            

    client.subscribe(topic)
    client.on_message = on_message


def run():
    client = connect_mqtt()
    subscribe(client)
    client.loop_forever()

if __name__ == '__main__':
    with open('New1.csv', 'w') as csv_file:
        csv_writer = csv.DictWriter(csv_file, fieldnames=fieldnames)
        csv_writer.writeheader()
    with open('New2.csv', 'w') as csv_file:
        csv_writer = csv.DictWriter(csv_file, fieldnames=fieldnames)
        csv_writer.writeheader()
    run()