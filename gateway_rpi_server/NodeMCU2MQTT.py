import sqlite3 # for sqlite 3
import datetime # we need this library to accomplish date and time functions
import paho.mqtt.client as mqtt  #for fetching published data in mqtt

import time
from time import sleep
import serial

#from Adafruit_IO import MQTTClient
# from Adafruit_IO import Client, Feed, Data

# adaclient = Client("ppvvrr","aio_vgeF24hPjhB69QtOMCYbGb1ZJpGI")

# MQTT connections & feeds
mqtt_address='192.168.43.218'    #for mqtt server address                   
mqtt_user=''                  #mqtt user id
mqtt_password=''              #mqtt password
mqtt_ppmtopic='/feeds/ppm'      #mqtt topic for pollut ion sensor
mqtt_temptopic='/feeds/temperature' #mqtt topic for temperature
mqtt_humitopic='/feeds/humidity' #mqtt topic for humidity
mqtt_current='/feeds/current'

# adafruit_io_server='io.adafruit.com'
# adafruit_io_key='aio_vgeF24hPjhB69QtOMCYbGb1ZJpGI'
# adafruit_io_username='ppvvrr'
# group_name='IoT'
# adafruit_io_serverport=1883

# def connected(client):
#     print("connected to Adafruit")
# 
# def disconnected(client):
#     print("disconnected from Adafruit")
#     sys.exit(1)

# def on_connect(client,userdta,flags,rc):
#     print(f"Connected with result code {rc}")

# def message(client,feed_id,payload):
#     client.publish(feed_id,payload)

# client=MQTTClient(adafruit_io_username,adafruit_io_key)
# client.on_connect=connected
# client.on_disconnect=disconnected
# # client.on_message=message
# client.connect()
# client.loop_background()
#client.connect(adafruit_io_server,1883,adafruit_io_username,adafruit_io_key,60)


#adaclient=Client(adafruit_io_username,adafruit_io_key)

# adafeed_ppm=adaclient.feeds('pollution')      #mqtt topic for pollut ion sensor
# adafeed_temp=adaclient.feeds('temperature') #mqtt topic for temperature
# adafeed_humi=adaclient.feeds('humidity') #mqtt topic for humidity
# adafeed_current=adaclient.feeds('Current')

# adafeed_ppm="/feeds/pollution"      #mqtt topic for pollut ion sensor
# adafeed_temp="/feeds/temperature" #mqtt topic for temperature
# adafeed_humi="/feeds/humidity" #mqtt topic for humidity
# adafeed_current="/feeds/current"




#Serial connection config
ser = serial.Serial(
    
    port='/dev/ttyS0',

    baudrate = 9600,

    parity=serial.PARITY_NONE,

    stopbits=serial.STOPBITS_ONE,

    bytesize=serial.EIGHTBITS,

    timeout=1             

 )

humvalue=''
temvalue=''
ppmval=''

#function for inserting pollution data in sqlite
def insertPollutionDataSQLite(node_id,ppm):   
    try:
        conn=sqlite3.connect('streetlight.db')  #connect with streetlight database
        cursor=conn.cursor()                    #open cursor for database connection
        print("sucessfully connected to DB")   
        
        times=datetime.datetime.now().strftime("%d-%b-%Y %H:%M:%S:%f") # form current time stamp 
        
        #sql_query string formation the string will be passed as parameter to excute sqlscript 

        
        sql_query= "DELETE from pollutionsensor;INSERT into pollutionsensor(node_id,ppm,timestamp) values("+str(node_id)+","+ppm+",'"+times+"');"
        
        agg_id=0
        temp=1
        hum=1
        aq=1
        wf=1
        tr=1
        f1=1
        f2=0
        timestamp="blaah"

        sql_query= "INSERT into alldata VALUES("+str(agg_id)+","+str(temp)+","+str(hum)+","+str(aq)+","+str(wf)+","+str(tr)+","+str(f1)+","+str(f2)+","+str(timestamp)+");"


        
        count = cursor.executescript(sql_query) # excute the script
        
        conn.commit()  # commit the sql connection
        cursor.close()  #close the cursor
        print("Pollution sensor record(s) inserted successfully")
        
    # expection block for database and sql query

    except sqlite3.Error as error:
        print("fail to insert record",error)
    #close all the connections
        
    finally:
        if conn:
            conn.close()# close the database connection
            print("database closed")


#function for inserting pollution data in sqlite
def insertTemperatureDataSQLite(node_id,temp,humi):   
    try:
        conn=sqlite3.connect('streetlight.db')  #connect with streetlight database
        cursor=conn.cursor()                    #open cursor for database connection
        print("sucessfully connected to DB")   
        
        times=datetime.datetime.now().strftime("%d-%b-%Y %H:%M:%S:%f") # form current time stamp 
        
        #sql_query string formation the string will be passed as parameter to excute sqlscript 
        
        sql_query= "delete from temperaturesensor;INSERT into temperaturesensor(node_id,temperature,humidity,timestamp) values("+str(node_id)+","+temp+","+humi+",'"+times+"');"
        
        count = cursor.executescript(sql_query) # excute the script
        
        conn.commit()  # commit the sql connection
        cursor.close()  #close the cursor
        print("Temperature sensor record(s) inserted successfully")
        
    # expection block for database and sql query

    except sqlite3.Error as error:
        print("fail to insert record",error)
    #close all the connections
        
    finally:
        if conn:
            conn.close()# close the database connection
            print("database closed")

#function for inserting pollution data in sqlite
def insertCurrentDataSQLite(node_id,curr):   
    try:
        conn=sqlite3.connect('streetlight.db')  #connect with streetlight database
        cursor=conn.cursor()                    #open cursor for database connection
        print("sucessfully connected to DB")   
        
        times=datetime.datetime.now().strftime("%d-%b-%Y %H:%M:%S:%f") # form current time stamp 
        
        #sql_query string formation the string will be passed as parameter to excute sqlscript 
        
        sql_query= "delete from currentsensor;INSERT into currentsensor(node_id,current,timestamp) values("+str(node_id)+","+curr+",'"+times+"');"
        
        count = cursor.executescript(sql_query) # excute the script
        
        conn.commit()  # commit the sql connection
        cursor.close()  #close the cursor
        print("Current sensor record(s) inserted successfully")
        
    # expection block for database and sql query

    except sqlite3.Error as error:
        print("fail to insert record",error)
    #close all the connections
        
    finally:
        if conn:
            conn.close()# close the database connection
            print("database closed")


# connect mqtt server for ppm data feed 
def on_connect(client,userdata,flags,rc):
#     print('mqtt connected with result code'+stc(rc))
    client.subscribe(mqtt_ppmtopic)
    client.subscribe(mqtt_temptopic)
    client.subscribe(mqtt_humitopic)

# ON sucessfull mqtt connection fetch ppm value and insert in sqlite database table
def on_message(client,userdata,msg):
    global humvalue,temvalue,ppmval
    if msg.topic=='/feeds/ppm':
        ppmval=msg.payload.decode()
        insertPollutionDataSQLite(1,ppmval)
        sleep(1)
        print('ppm ',ppmval)
#         client.publish('ppm',ppmval)
#         sleep(1)
        #adaclient.send(adafeed_ppm.key,ppmval)
        #adaclient.send('pollution',humvalue)
    if msg.topic=='/feeds/humidity':
        humvalue=msg.payload.decode()
        print('humidity ',humvalue)
#         client.publish('humidity',humvalue)
        sleep(1)
        #adaclient.send(adafeed_humi.key,humvalue)
        #adaclient.send('humidity',humvalue)
    if msg.topic=='/feeds/temperature':
        temvalue=msg.payload.decode()
        insertTemperatureDataSQLite(1,temvalue,humvalue)
        sleep(1)
        print('temperature ',temvalue)
#         client.publish('temperature',temvalue)
        sleep(1)
        #adaclient.send(adafeed_temp.key,temvalue)
        #adaclient.send('temperature',temvalue)
#         print("PPM, Temperature, Humidity data passed to ADAfruit succcessfully")
    
    # Read serial data through Xbee
    sval=ser.readline().strip() 
    print(sval)
    if sval.isdigit():
        xenc=sval.decode()
        inval=round(((float(xenc)/1024)-.5)/0.185,2)
        print("Current ",inval)
        insertCurrentDataSQLite(2,str(inval))
        sleep(5)
#         client.publish('Current',inval)
#         sleep(2)
        #adaclient.send(adafeed_current.key,inval)
        #adaclient.send('Current',inval)
#         print("Current data passed to ADAfruit successfully")
    
    
    

def main():
    mqtt_client=mqtt.Client()
    mqtt_client.username_pw_set(mqtt_user,mqtt_password)
    mqtt_client.on_connect=on_connect
    mqtt_client.on_message=on_message
       
#     adaclient=MQTTClient(adafruit_io_server,adafruit_io_serverport,adafruit_io_username,adafruit_io_key)

    
#     adaclient.on_connect = connected
#     adaclient.on_disconnect = disconnected
#     adaclient.on_message = message
    
#    adaclient.connect()
    
    #connect to mqtt server
    mqtt_client.connect(mqtt_address,1883)
    mqtt_client.loop_forever()  #listen for published data forever

if __name__=='__main__':
    print('MQTT to InfluxDB bridge')
    main()
