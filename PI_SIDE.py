import serial
import time
import socket 
import threading
from queue import Queue

HEADER = 64
PORT = 5051
SERVER = "192.168.1.10"     #server = "192.168.1.10"

FORMAT = 'utf-8'
DISCONNECT_MESSAGE="!DIS"
q=Queue()

def arm(coord):
    
    arduino_serial = serial.Serial('/dev/ttyAMA0', 9600, timeout=1)
    arduino_serial.flush()

    #x=70
    y=300
    #z=200
    twist_angle=50
    x=str(x)+"\n"
    y=str(y)+"\n"
    z=str(z)+"\n"
    twist_angle=str(twist_angle)+"\n"
    finish_flag="finished"+"\n"

    first=1

    def send_coordinates():
        def decode_msg():
            x=coord[coord.find("[")+1:coord.find(".")]+"\n"
            coord=coord.replace(coord[coord.find("[")+1:coord.find(".")+2],"")

            z=coord[coord.find("[")+1:coord.find(".")]+"\n"
            coord=coord.replace(coord[coord.find("[")+1:coord.find(".")+2],"")
            return x,z

        x,z=decode_msg()
        arduino_serial.write(x.encode('utf-8'))
        arduino_serial.write(y.encode('utf-8'))
        arduino_serial.write(z.encode('utf-8'))
        arduino_serial.write(twist_angle.encode('utf-8'))
        arduino_serial.write(finish_flag.encode('utf-8'))
        
    while True:
        line = arduino_serial.readline().decode('utf-8').rstrip()
        if line=="ready" or first:
            send_coordinates()
            first=0
            line="not_ready"
        break
##############################################################################################
def flag_sender(q):
    port=PORT+1
    ADDR= ("192.168.233.251",port)                                          #laptop IP
    client= socket.socket(socket.AF_INET,socket.SOCK_STREAM)                #create a new socket wit default settings
        
    while True:
        try:
            client.connect(ADDR)                                            #connect to the socket at ADDR
            break
        except:
            print("[FLAG_SERVER]ERROR:Can't connect (Re-enter IP or port),(an another server exists on port),(server didn't start)")
            time.sleep(1)
            
        
    print ("[FLAG_SERVER]connected to Laptop")
    
    def send(msg):                                                          #defining what to send
        message = msg.encode(FORMAT)                                        #encode the message
        msg_length = len(message)                                           #get message length
        send_length = str(msg_length).encode(FORMAT)                        #encode message length 
        send_length += b' ' * (HEADER - len(send_length))                   #prepare the message length to be send first ,"6    " if HEADER=5
        client.send(send_length)                                            #defining what to send
        client.send(message)                                                #send message itself
        print("[FLAG_SERVER]sent msg to Laptop:"+msg)

    
    while True:
        send("scan")
        while True:                 #waiting for coordinates to be recived
            if q.empty():
                pass
            else:
                break
        
        while not q.empty():                 #feeding coordinates to arm
            cor=q.get()
            if cor=="empty":
                print("empty recived")
                pass
            else:
                arm(cor)


##############################################################################################
def cord_listener(q):
    ADDR= ("192.168.233.100",PORT)                                          #PI IP
    server= socket.socket(socket.AF_INET,socket.SOCK_STREAM)                #create a new socket wit default settings
    try:
        server.bind(ADDR)                                                   #Bind the socket to address
    except:
        print("[FLAG_SERVER]ERROR:Can't connect (Re-enter IP or port),(an another server exists on port),(server didn't start)")
        

    def handle_client(conn, addr):
        print("[COORDINATES_SERVER]Laptop "+str(addr)+" is connected")
        connected = True
        while connected:
            msg_length = conn.recv(HEADER).decode(FORMAT)                   #receive incoming message length ,assume the first message length = HEADER
            if msg_length:                                                  #if message length is defined
                msg_length = int(msg_length)                                #convert message length to int
                msg = conn.recv(msg_length).decode(FORMAT)                  #receive incoming message
                if msg == DISCONNECT_MESSAGE:                               #disconnect condition
                    connected=False
                    print("[COORDINATES_SERVER]Laptop "+str(addr)+" DISCONNECTED")
                    #quit()
                else:
                    print("[COORDINATES_SERVER]msg recived from Laptop "+str(addr)+": "+msg)
                    q.put(msg)                                              #store coordinates in flag queue 2
        conn.close

    def start():
        server.listen()                                                     #server is listen for incoming connection on ADDR
        print("[COORDINATES_SERVER]server is listening on "+SERVER)
        while True:
            conn, addr = server.accept()                                    #accept the incoming connection 
            thread=threading.Thread(target=handle_client,args=(conn,addr))  #run flag_listener in a separate thread for the incoming connection
            thread.start()

    print("[COORDINATES_SERVER]server is starting")
    start()

thread1 = threading.Thread(target=flag_sender, args=(q,))                   #run flag_sender in a separate thread
thread2 = threading.Thread(target=cord_listener, args=(q,))                 #run cord_listener in a separate thread

thread1.start()
thread2.start()