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
        # with open('document.csv','a',newline='') as fd:
        #     spamwriter = csv.writer(fd)
        #     temp = str(msg.payload.decode()).split(',')
            
        #     spamwriter.writerow([temp[i] for i in range(len(temp))])
            

    client.subscribe(topic)
    client.on_message = on_message


def run():
    client = connect_mqtt()
    subscribe(client)
    client.loop_forever()


if __name__ == '__main__':
    run()