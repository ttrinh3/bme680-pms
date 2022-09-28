[x] add pms\
[x] add bme680\
[ ] add lora\
[x] test the switch\
[ ] reduce the redudancy gd
[ ] try to do both 280 and bme680 at the same time\
you probably only need to do change the i2c bus since you are already using different pins\
but you will probably need to switch around the pms so that it isn't done twice\ 
same with time\
by the way, youre also going to do timer method, not sntp because the nodes might not have internet 
connection\

if you DO use timer then you must plug in the microcontrollers at the same time
