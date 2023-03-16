#!/usr/bin/env python3

""" A simple continuous receiver class. """
from time import sleep
from SX127x.LoRa import *
from SX127x.LoRaArgumentParser import LoRaArgumentParser
from SX127x.board_config import BOARD
import threading
import paho.mqtt.client as mqttClient
import datetime

####################     LORA            ##############################################################
class message:
    
    ID=0
    digital=0
    analogconverted=0.0
    analog=[0,0]
    future=[0,0,0,0]


    def pack():
        pass

    def extract():
        pass

    def __sizeof__(self) -> int:
        print("content of message")
        pass


class LoRaRcvCont(LoRa):
    FLAG_RX_COMPLETED=0
    FLAG_TX_COMPLETED=0
    payload=[]
    def __init__(self, verbose=False):
        super(LoRaRcvCont, self).__init__(verbose)
        self.set_mode(MODE.SLEEP)
        self.set_dio_mapping( [0, 0, 0, 0, 0, 0])


    def on_rx_done(self):
        self.set_mode(MODE.STDBY)
        print("\nLORA GOT DATA ",end="")
        self.clear_irq_flags(RxDone=1)
        self.payload = self.read_payload(nocheck=True)
        print(" PAYLOAD LENGTH =",len(self.payload))
        for i in self.payload:
            print(hex(i),end= " ")
        print("\n")
        self.set_mode(MODE.SLEEP)
        self.reset_ptr_rx()
        self.FLAG_RX_COMPLETED=1
        
        # BOARD.led_off()
        # self.set_mode(MODE.RXCONT)
        ##############################

  

    def transmit(self,payload):
        self.set_dio_mapping( [1, 0, 0, 0, 0, 0])
        self.write_payload(payload)
        self.set_mode(MODE.TX)


    def on_tx_done(self):
        print("\nLORA TX...DONE")
        self.set_mode(MODE.STDBY)
        self.clear_irq_flags(TxDone=1)
        self.set_dio_mapping( [0, 0, 0, 0, 0, 0])
        self.FLAG_TX_COMPLETED=1




    def on_cad_done(self):
        print("\non_CadDone")
        print(self.get_irq_flags())

    def on_rx_timeout(self):
        print("\non_RxTimeout")
        print(self.get_irq_flags())

    def on_valid_header(self):
        print("\non_ValidHeader")
        print(self.get_irq_flags())

    def on_payload_crc_error(self):
        print("\non_PayloadCrcError")
        print(self.get_irq_flags())

    def on_fhss_change_channel(self):
        print("\non_FhssChangeChannel")
        print(self.get_irq_flags())

    def start(self):
        self.reset_ptr_rx()
        self.set_mode(MODE.RXCONT)
        while True:
            sleep(.5)
            rssi_value = self.get_rssi_value()
            status = self.get_modem_status()
            # sys.stdout.flush()
            # sys.stdout.write("\rRSSI=%d" % (rssi_value))
            print("rssi=",rssi_value)


def lora_process():
    FLAG_IS_IN_RXMODE=0
    BOARD.setup()
    parser = LoRaArgumentParser("DEADBEEF")
    commandbuff=[]
    lora = LoRaRcvCont(verbose=False)
    args = parser.parse_args(lora)
    lora.set_mode(MODE.STDBY)
    lora.set_freq(433.0)
    lora.set_pa_config(pa_select=1)
    lora.reset_ptr_rx()

    print(lora)
    assert(lora.get_agc_auto_on() == 1)


    try:

        while True:
            print("\nlooop-----------------------")
            FLAG_IS_IN_RXMODE=1
            lora.set_mode(MODE.RXCONT)
            while (FLAG_IS_IN_RXMODE and not lora.FLAG_RX_COMPLETED ):
                # sys.stdout.flush()
                # sys.stdout.write("\rLORA receiving")
                # print("LORA receiving")
                # stalling here till we receive some data
                # you exit this when  lora.FLAG_RX_COMPLETED gets high
                pass

            sleep(0.5)
            
            # FLAG_IS_IN_RXMODE=0 # out of receive mode

            if(lora.FLAG_RX_COMPLETED):
                print("\ngonna do transmit stuff")
                
                FLAG_IS_IN_RXMODE=0
                # we check for any data on payload
                # if(lora.payload[0]== 0x48):
                try :
                    if(lora.payload[11]==0x00):
                        data=[0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF]
                    else :
                        data=[0xFF,0x00,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF]
                except e:
                    print("sending first data")
                    data=[0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF]
        
                lora.transmit(data)
                print("SEND =",end=" ")
                for i in data:
                    print(hex(i),end=" ")
                while not lora.FLAG_TX_COMPLETED:
                    pass

                #  resetting the class variable
                lora.FLAG_RX_COMPLETED=0 
                lora.FLAG_TX_COMPLETED=0
                



    except KeyboardInterrupt:
        sys.stdout.flush()
        print("")
        sys.stderr.write("KeyboardInterrupt\n")
    finally:
        sys.stdout.flush()
        lora.set_mode(MODE.SLEEP)
        BOARD.teardown()
    pass
###########################################################################LORA END


if __name__=="__main__":

    LORATHREAD=threading.Thread(target=lora_process,daemon=True)
    LORATHREAD.start()
    while True:
        print("main thread is doing something else ")
        sleep(2)
    threading.shutdown()

