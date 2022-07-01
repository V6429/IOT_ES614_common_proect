# first of all import the socket library
import pickle
import socket			
import sys
import sqlite3,datetime
from colorama import Fore,Back,Style
color_matrix = [Fore.RED,Fore.BLUE,Fore.GREEN,Fore.MAGENTA,Fore.YELLOW,Fore.CYAN,Fore.LIGHTGREEN_EX,Fore.WHITE]
debug=1
RUNTHREADS=True


port = 6565	


####################################DBMS SPECIFIC PARAMETERS
dbname="smart_home.db"



if __name__=="__main__":

    # next create a socket object
    s = socket.socket()		
    print ("Socket successfully created")
    # reserve a port on your computer in our
    # case it is 12345 but it can be anything

    # Next bind to the port
    # we have not typed any ip in the ip field
    # instead we have inputted an empty string
    # this makes the server listen to requests
    # coming from other computers on the network

    s.bind(('', port))		
    print ("socket binded to %s" %(port))
    # put the socket into listening mode
    s.listen(5)	
    print ("socket is listening")		
   
    while True:
        try :

            sql_conn=sqlite3.connect(dbname)
            sql_cursor = sql_conn.cursor()
            break
        except Exception as e:
            print(e)
            print("WAS NOT ABLE TO CONNECT DATABASE")



    # a forever loop until we interrupt it or
    # an error occurs
    while True:
        try :
        # Establish connection with client.
            c, addr = s.accept()	
            print ('Got connection from', addr )

            while True:
                try:
                    data=c.recv(2000)
                    if data:
                        data_dict=pickle.loads(data)
                        print(data_dict)
                        sql_query= "INSERT into Data VALUES('" \
                        +str(data_dict["AGG_ID"])+"',"+str(data_dict["nodem_temp"])+","+str(data_dict["nodem_hum"])+"," \
                        +str(data_dict["loranode_gas"])+","+str(data_dict["loranode_waterflow"]) \
                        +","+str(data_dict["zignode_temp"])+",'"+str(data_dict["timestamp"])+"');" 
                        print(sql_query)
                        sql_cursor.execute(sql_query)
                        sql_conn.commit()              
                except KeyboardInterrupt:
                    c.close()
                    # Breaking once connection closed
                    sys.exit(0)
                    break

        except Exception as error:
            print(error)
            s.close()
            print("something went wrong")
            sys.exit(0)
