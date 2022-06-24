# Import Module
from tkinter import *
import paho.mqtt.client as mqttClient
import time 

 
def on_connect(client, userdata, flags, rc):
    if rc == 0:
        print("Connected to broker")
        global Connected                #Use global variable
        Connected = True                #Signal connection 
    else:
        print("Connection failed")
        





def switch():
	global is_on
	
	# Determine is on or off
	if is_on:
		on_button.config(image = off)

		my_label.config(text = "The Switch is Off",
						fg = "grey")
		
		
		is_on = False
	else:
                if(connected):# add a try catch here
                    client.publish("/feeds/relay1","test test test")
                    
		on_button.config(image = on)
		my_label.config(text = "The Switch is On", fg = "green")
		is_on = True
	

##if __name__=="__main__":
print("helooo")
Connected = False   #global variable for the state of the connection
broker_address= "192.168.43.153"
port = 1883
user = ""
password = ""
    


# Create Object
root = Tk()
my_label = Label(root,text = "The Switch Is On!",fg = "green",font = ("Helvetica", 32))
my_label.pack(pady = 20)
# Add Title
root.title('On/Off Switch!')
# Define Our Images
on = PhotoImage(file = "on.png")
off = PhotoImage(file = "off.png")
# Create A Button
on_button = Button(root, image = on, bd = 0,command = switch)
on_button.pack(pady = 50)


# Add Geometry
root.geometry("500x300")
# Keep track of the button state on/off
#global is_on
is_on = True
# Create Label
# Execute Tkinter
root.mainloop()

print("heloooo")
client = mqttClient.Client("Python")               #create new instance
client.username_pw_set(user, password=password)    #set username and password
client.on_connect= on_connect                      #attach function to callback
client.connect(broker_address, port=port)          #connect to broker
 
client.loop_start()        #start the loop
 
while Connected != True:    #Wait for connection
    
    time.sleep(0.1)
    
try:
    while True:
 
        value = input('Enter the message:')
        client.publish("/feeds/relay1",value)
 
except KeyboardInterrupt:
 
    client.disconnect()
    client.loop_stop()
    
    

