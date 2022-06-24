from tkinter import *
import RPi.GPIO as GPIO
GPIO.setmode(GPIO.BOARD)
GPIO.setup(7,GPIO.OUT)
GPIO.setup(8,GPIO.OUT)
GPIO.setup(9,GPIO.OUT)
GPIO.setup(10,GPIO.OUT)

window = tk()
window.title(‘Raspberry GUI Home Automation System’)
from tkinter import *
from paho.mqtt import client as mqtt_client
import json
 
broker = 'iot.reyax.com'
port = 1883
topic = "api/request"
topic_sub = "api/notification/37/#"
# generate client ID with pub prefix randomly
client_id = 'your client id'
username = 'your username'
password = 'your password'
deviceId = "your deviceId"
 
 
def connect_mqtt():
    def on_connect(client, userdata, flags, rc):
        if rc==0:
            print("Successfully connected to MQTT broker")
        else:
            print("Failed to connect, return code %d", rc)
 
 
    client = mqtt_client.Client(client_id)
    client.username_pw_set(username, password)
    client.on_connect = on_connect
    client.connect(broker, port)
    return client
 
def publish(client,status):
    # msg = f"messages: {msg_count}"
    msg = "{\"action\":\"command/insert\",\"deviceId\":\""+deviceId+"\",\"command\":{\"command\":\"LED_control\",\"parameters\":{\"led\":\""+status+"\"}}}"
    # msg = '{"action":"command/insert","command":{"id":432436060,"command":"LED_control","timestamp":"2021-03-24T00:19:44.418","lastUpdated":"2021-03-24T00:19:44.418","userId":37,"deviceId":"s3s9TFhT9WbDsA0CxlWeAKuZykjcmO6PoxK6","networkId":37,"deviceTypeId":5,"parameters":{"led":"on"},"lifetime":null,"status":null,"result":null},"subscriptionId":1616544981034531}'
    result = client.publish(topic, msg)
    # result: [0, 1]
    status = result[0]
    if status == 0:
        print(f"Send `{msg}` to topic `{topic}`")
    else:
        print(f"Failed to send message to topic {topic}")
 
 
def subscribe(client: mqtt_client):
    def on_message(client, userdata, msg):
        #print(f"Recieved '{msg.payload.decode()}' from '{msg.topic}' topic")
        y = json.loads(msg.payload.decode())
        temp = str(y["notification"]["parameters"]["temp"])
        hum = str(y["notification"]["parameters"]["humi"])
        print("temperature: ",temp,", humidity:",hum)
        temp_label.config(text=temp+" °C",
                        fg="black")
 
        hum_label.config(text=hum + "  %",
                          fg="black")
 
 
 
 
    client.subscribe(topic_sub)
    client.on_message = on_message
 
 
window = Tk()
window.title("MQTT Dashboard")
window.geometry('395x675')
window.resizable(False,False)
window.configure(bg="white")
canvas = Canvas(window, bg="white", width=395,height=135)
canvas.place(x=0,y=0)
img = PhotoImage(file="logo.png")
canvas.create_image(0,0,anchor=NW,image=img)
 
canvas2 = Canvas(window,width=100,height=100)
canvas2.place(x=50,y=165)
img2 = PhotoImage(file="temp2.png")
canvas2.create_image(0,0,anchor=NW,image=img2)
 
canvas3 = Canvas(window,width=100,height=100)
canvas3.place(x=50,y=295)
img3 = PhotoImage(file="hum.png")
canvas3.create_image(0,0,anchor=NW,image=img3)
 
is_on = False
 
 
 
# Create Label
my_label = Label(window,
                 text="OFF!",
                 bg="white",
                 fg="grey",
                 font=("Helvetica", 32))
my_label.place(x=180,y=430)
 
def switch():
    global is_on
    if is_on:
        on_button.config(image=off)
        my_label.config(text="OFF!",
                        fg="grey")
        print("LED is off now")
        publish(client,"0")
        is_on=False
    else:
        on_button.config(image=on)
        my_label.config(text="ON!",
                        fg="red")
        print("LED is On now")
        publish(client,"1")
        is_on=True
 
# Define Our Images
on = PhotoImage(file="on1.png")
off = PhotoImage(file="off1.png")
 
# Create A Button
on_button = Button(window, image=off, bd=0,
                   command=switch)
on_button.place(x=50,y=420)
 
canvas4 = Canvas(window, bg="white", width = 395, height = 135)
canvas4.place(x=0,y=545)
img4 = PhotoImage(file="hv.png")
canvas4.create_image(0,0, anchor=NW, image=img4)
 
# Create Label
temp_label = Label(window,
                 text=" °C",
                 bg="white",
                 fg="black",
                 font=("Helvetica", 32))
 
temp_label.place(x=180,y=190)
 
# Create Label
hum_label = Label(window,
                 text="  %",
                 bg="white",
                 fg="black",
                 font=("Helvetica", 32))
 
hum_label.place(x=180,y=325)
client = connect_mqtt()
subscribe(client)
client.loop_start()
 
 
window.mainloop()
client.loop_stop()
window.geometry(‘800x500’)

def relay1_on():
    GPIO.output(7,True)
    relay1_status.configure(text = ‘ON’)
    button_relay1.configure(command = relay1_off)

def relay1_off():
    GPIO.output(7,Flase)
    relay1_status.configure(text = ‘OFF’)
    button_relay1.configure(command = relay1_on)

def relay2_on():
    GPIO.output(8,True)
    relay2_status.configure(text = ‘ON’)
    button_relay2.configure(command = relay2_off)

def relay2_off():
    GPIO.output(8,Flase)
    Relay2_status.configure(text = ‘OFF’)
    button_relay2.configure(command = relay2_on)

def relay3_on():
    GPIO.output(9,True)
    relay3_status.configure(text = ‘ON’)
    button_relay3.configure(command = relay3_off)

def relay3_off():
    GPIO.output(9,Flase)
    relay3_status.configure(text = ‘OFF’)
    button_relay3.configure(command = relay3_on)

def relay4_on():
    GPIO.output(10,True)
    Relay4_status.configure(text = ‘ON’)
    button_relay4.configure(command = relay4_off)

def relay4_off():
    GPIO.output(10,Flase)
    relay4_status.configure(text = ‘OFF’)
    button_relay4.configure(command = relay4_on)

def cleanup():
    GPIO.output(7,Flase)

    GPIO.output(8,Flase)
    GPIO.output(9,Flase)
    GPIO.output(10,Flase)

    relay1_status = Label(window, relief =’flat’, width = 4)
    relay2_status = Label(window, relief =’flat’, width = 4)
    relay3_status = Label(window, relief =’flat’, width = 4)
    relay4_status = Label(window, relief =’flat’, width = 4)

    button_relay1 = Button(window)
    button_relay2 = Button(window)
    button_relay3 = Button(window)
    button_relay4 = Button(window)

    button_relay1.configure(text='RELAY1',command=relay1_on)
    button_relay2.configure(text='RELAY2',command=relay2_on)
    button_relay3.configure(text='RELAY3',command=relay3_on)
    button_relay4.configure(text='RELAY4',command=relay4_on)

    relay1_status.configure(text='OFF') 
    relay2_status.configure(text='OFF') 
    relay3_status.configure(text='OFF') 
    relay4_status.configure(text='OFF') 

    button_relay1.grid(row=1,column=1,columnspan=1)
    red_led_relay1.grid(row=1,column=2,padx=10)

    button_relay2.grid(row=1,column=1,columnspan=1)
    red_led_relay2.grid(row=1,column=2,padx=10)

    button_relay3.grid(row=1,column=1,columnspan=1)
    red_led_relay3.grid(row=1,column=2,padx=10)

    button_relay4.grid(row=1,column=1,columnspan=1)
    red_led_relay4.grid(row=1,column=2,padx=10)


atexit.register(cleanup)
window.mainloop()