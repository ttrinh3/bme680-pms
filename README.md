# Wildfire Project Summary
The idea of this project is to provide a tailored low cost IoT solution for forest wildfire prevention. 
## What?
The system is a sensor network, which consists of 3 main parts: 1) gathering nodes, 2) central node, 3) central server.
The gathering nodes collect data through an array of sensors and transmit them to a central node. The central node processes the data and decides whether or not the received data is indicative of a fire. The central server is an optional component that provides access to cloud-related services such as hosting databases or machine learning platforms or something else idk we didn't get there yet. 

## How?

Because this project was initiated by uninspired collegiate individuals, the budget is basically pocket change, and the design will reflect that. 

The idea is to capture short-term/immediate environmental changes brought upon by fires, and this job would be done by the below sensors:
1) BME680
The BME680 is sensitive to pressure, gas, temperature and humidity changes.
2) PMS5003 PM Sensor
This detects particulate matter

Fires would ideally alter these values in a specific manner, and a well designed algorithm or machine learning model would be sensitive to these.

We chose LoRa and MQTT as methods of communication. The LoRa module used is the rfm95x. 

## Milestones

The default branch contains code for a single esp32s3 board, which interfaces with the rfm, bme680 (AND 280 just for comparison purposes) and the PMS. The central node branch contains code for another esp32s3 to receive from the previous node and transmit to a remote mqtt server. That's pretty much it. Just basic sensor operation and transmission. Most of the code wasn't written from scratch but mostly pieced together from other open source projects. The top-secret branch contains the beginnings of RISCV ULP bit banged i2c interfacing for BME680




