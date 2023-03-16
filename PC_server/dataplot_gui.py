import tkinter as tk
from tkinter import Canvas, ttk
#to import backend matplot libraries
import matplotlib
from numpy import average
matplotlib.use("TkAgg")
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg, NavigationToolbar2Tk
from matplotlib.figure import Figure
import sqlite3
# import paho.mqtt.client as mqttclient

# ################MQTT
# broker_address= 'localhost' # "192.168.43.140"
# port = 1883
# user = "rpi"
# password = "open1234"
# MQTT_connected=False

# ################################# MQTT TOPICS and settings
# ###NODEM
# nodem_topic_relay1      ="/feeds/nodem_relay1"
# nodem_topic_relay2      ="/feeds/nodem_relay2"
# nodem_topic_relay3      ="/feeds/nodem_relay3"
# nodem_topic_relay4      ="/feeds/nodem_relay4"



# def connectMQTTcallback(client, userdata, flags, rc):
#     if rc == 0:
#         print(Fore.GREEN,"Connected to MQTT broker",Fore.RESET)
#         global Connected                #Use global variable
#         Connected = True                #Signal connection 
#         ##########################SUBSCRIBED CONTENT
#         client.subscribe(nodem_topic_temp)
#         client.subscribe(nodem_topic_hum)

#         client.subscribe(loranode_topic_control)
#         #############################################
#         return True
#     else:
#         print(Fore.RED,"MQTT Connection failed",Fore.RESET)
#         Connected = False 
#         return False

# def connectMQTT():
#     client = mqttClient.Client("GATEWAY-"+str(random.randint(2000,9999)))               #create new instance
#     client.username_pw_set(user, password=password)                                     #set username and password
#     client.on_connect= connectMQTTcallback                                              #attach function to callback
#     client.on_message = on_message
#     client.connect(broker_address, port=port)                                           #connect to broker
#     client.loop_start()                                                                 #start the loop
#     return client

# def MQTT_thread():
#     i=0
#     while RUNTHREADS:
#         try :
#             #DATA BASE TODO cleanup
#             conn=sqlite3.connect(dbname)
#             cursor = conn.cursor()
#             mqttclient=connectMQTT()
#             while Connected !=True:
#                 sleep(1)
#             try: #if connected
#                 while True:
#                     #NODEM
#                     # mqttclient.publish(nodem_topic_relay1,nodem_relays["Relay1"])
#                     # mqttclient.publish(nodem_topic_temp,nodem_temp)
#                     # mqttclient.publish(nodem_topic_hum,nodem_hum)
#                     mqttclient.publish(nodem_topic_relay1,1)
#                     # mqttclient.publish(nodem_topic_relay2,0)
#                     # mqttclient.publish(nodem_topic_relay3,1)
#                     # mqttclient.publish(nodem_topic_relay4,1)
#                     return
#             except:
#                 print(Fore.RED,"MQTT CONNECTION FAILED",Fore.RESET)
#                 return
    

y_axis = []


def close():
    quit()

class frames(tk.Tk):
    def __init__(self, *args, **kwargs):
        tk.Tk.__init__(self, *args, **kwargs)
        container = tk.Frame(self) 
        container.pack(side="top", fill="both",expand=True)
        container.grid_rowconfigure(0, weight=1)
        container.grid_columnconfigure(0, weight=1)
        

        self.frames = {}
        for F in (startPage, pageOne, pageTwo):   
            frame = F(container,self) 
            self.frames[F] = frame
            frame.grid(row=0,column=0,sticky="nsew")
        self.show_frame(startPage)

    def show_frame(self, cont):
        frame= self.frames[cont]        
        frame.tkraise()

#=========================================================================

class startPage(tk.Frame):
    def __init__(self, parent,controller):
        tk.Frame.__init__(self,parent)
        label = tk.Label(self, text="HOME1 - DASHBOARD")
        label.pack(pady=50,padx=10)
        
        button1 = ttk.Button(self,text="View temperature Plot",
                            command=lambda: controller.show_frame(pageOne))
        button1.pack()

        button2 = ttk.Button(self,text="Remote control",
                            command=lambda: controller.show_frame(pageTwo))
        button2.pack()

        button3 = ttk.Button(self,text="close",
                            command= close)
        button3.pack(pady=150,padx=50)

        

#========================================================================================================

class pageOne(tk.Frame):
    def __init__(self, parent, controller):
        tk.Frame.__init__(self,parent)
        label = tk.Label(self, text="Temperature Visualization \n (Based on last 10 datas)")
        label.pack(pady=10,padx=10)
        
        button3 = tk.Button(self,text="Back to Home Window",
                            command=lambda: controller.show_frame(startPage))
        button3.pack()

        #create a db or connect to one 
        conn = sqlite3.connect('smart_home.db')
        #create cursor
        cursor = conn.cursor()

        #query the database
        # cursor.execute("SELECT *, oid FROM Data;")
        # records=cursor.fetchall()
        records = []
        for i in range(0,10,1):
            cursor.execute("SELECT * FROM Data WHERE OID=(SELECT MAX(OID) FROM Data)-"+str(i)+";")
            records.append(cursor.fetchone())
        
        print(records)
        temp = []
        #Display the captured data in GUI
        print_records = ''
        for record in records:  
            print_records = float(record[1])  
            #print(print_records)
            temp.append(print_records) 
        #print(temp)
        # query_label = Label(root, text= print_records)
        # query_label.grid(row=6, column=0, columnspan=2)
        
        #commit changes
        conn.commit()

        #close connection
        conn.close()

        y_axis = temp
        x_axis = [1,2,3,4,5,6,7,8,9,10]


        f= Figure(figsize=(5,5),dpi= 100)
        a= f.add_subplot(111)
        a.plot(x_axis,y_axis)

        canvas= FigureCanvasTkAgg(f,self)
        #canvas.show()
        canvas.get_tk_widget().pack(side=tk.TOP, fill=tk.BOTH, expand =True)

        toolbar = NavigationToolbar2Tk(canvas, self)
        toolbar.update()
        canvas._tkcanvas.pack(side=tk.TOP, fill=tk.BOTH, expand=True)

        average= sum(y_axis)/len(y_axis)
        label2 = tk.Label(self, text="Mean Temperature : "+ str(average) )
        label2.pack(pady=10,padx=10)





class pageTwo(tk.Frame):
    def __init__(self, parent, controller):
        tk.Frame.__init__(self,parent)
        label = tk.Label(self, text="PAGE -2", )
        label.pack(pady=10,padx=10)
        
        button4 = ttk.Button(self,text="Back to Home Window",
                            command=lambda: controller.show_frame(startPage))
        button4.pack(pady=10,padx=10)

        label1 = tk.Label(self, text="PAGE -2 \n Remote Control ", )
        label1.pack(pady=10,padx=10)

        button5 = ttk.Button(self,text="relay on",
                            )
        button5.pack() 
        
        button6 = ttk.Button(self,text="relay off",
                            command=lambda: controller.show_frame(startPage))
        button6.pack()
        # button6 = ttk.Button(self,text="Back to Home Window",
        #                     command=lambda: controller.show_frame(startPage))
        # button6.pack()
        
        # button7 = ttk.Button(self,text="Back to Home Window",
        #                     command=lambda: controller.show_frame(startPage))
        # button7.pack()

        # button8 = ttk.Button(self,text="Back to Home Window",
        #                     command=lambda: controller.show_frame(startPage))
        # button8.pack()

              

################################################################

app= frames()
app.mainloop()





