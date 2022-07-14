from multiprocessing import dummy
from operator import le
from queue import Queue
import queue
import cv2
import numpy as np
import socket 
import threading
import time

q=Queue()
qf=Queue()

HEADER = 64
PORT = 5051
FORMAT = 'utf-8'
DISCONNECT_MESSAGE="!DIS"
FONT = cv2.FONT_HERSHEY_PLAIN 

def process_img(q,qf):
    d=6.5/0.3125
    net = cv2.dnn.readNet(r'C:\Users\Muhammed\Desktop\yolov3_training_last (3).weights',r'C:\Users\Muhammed\Desktop\yolov3_training.cfg')
    classes = ["Ripe","Unripe","Rotten"]

    def arm_XYZ(x, y, w, h):        #at 10cm apple width(w)=500
        #arm_z=10*500/w#cm        #arm_z
        ref_x=width/2
        ref_y=height-height*0.0
        arm_x=(x+w/2-ref_x)              #x limit from arm needed??
        arm_y=(ref_y-y-h/2)-160
                                 
        z=(width*d/w*10)+90/0.3125
        label="z="+str(round(z*0.3125, 2))+"mm"

        cv2.putText(img, label, (x, y+h), FONT, 2, [0,204,204], 2)
        cv2.line(img,(int(ref_x),int(ref_y)),(int(x+w/2),int(y +h/2)),[204,0,204],1)

        return [arm_x,arm_y,z,h]

    while True:

        if qf.get()=="scan":                                                #start detection on execution on reciving scan flag
            
            try:
                layer_names = net.getLayerNames()
                output_layers = [layer_names[i - 1] for i in net.getUnconnectedOutLayers()]
                colors = np.random.uniform(0, 255, size=(len(classes), 3))

                x=0
                while x<=1:
                    x=x+1
                    webcam = cv2.VideoCapture("http://192.168.233.100:5050/stream.mjpg")
                # Loading image
                ret,img = webcam.read()
                height, width, channels = img.shape
                # Detecting objects
                blob = cv2.dnn.blobFromImage(img, 0.00392, (416, 416), (0, 0, 0), True, crop=False)

                net.setInput(blob)
                outs = net.forward(output_layers)

                # Showing informations on the screen
                class_ids = []
                confidences = []
                boxes = []
                for out in outs:
                    for detection in out:
                        scores = detection[5:]
                        
                        class_id = np.argmax(scores)
                        
                        confidence = scores[class_id]
                        if confidence > 0.4:
                            #print("confidence="+str(confidence))
                            # Object detected
                            #print(class_id)
                            center_x = int(detection[0] * width)
                            center_y = int(detection[1] * height)
                            w = int(detection[2] * width)
                            h = int(detection[3] * height)

                            # Rectangle coordinates
                            x = int(center_x - w / 2)
                            y = int(center_y - h / 2)

                            boxes.append([x, y, w, h])
                            confidences.append(float(confidence))
                            class_ids.append(class_id)

                indexes = cv2.dnn.NMSBoxes(boxes, confidences, 0.5, 0.4)
                #print(indexes)
                                              
                duplicant =""
                for i in range(len(boxes)):
                    if i in indexes:
                        x, y, w, h = boxes[i]
                        label = str(classes[class_ids[i]])                          #give each class a unique colour

                        if(class_ids[i]==0):                                        
                            color = [102, 102, 255]
                            ripe_cord=np.round_(np.multiply(arm_XYZ(x, y, w, h),0.3125))
                            if str(ripe_cord) != duplicant:                     #prevent sending the same coordiate twice
                                q.put(ripe_cord)
                                duplicant=str(ripe_cord)
                        elif(class_ids[i]==1):
                            color = [102, 255, 102]
                        else:
                            color = [0, 25, 51]

                        cv2.rectangle(img, (x, y), (x + w, y + h), color, 2)        #print the box on frame
                        cv2.circle(img,(x+int(w/2),y+int(h/2)),3,[0, 179, 255],-1)  #print center of the box on the frame
                        cv2.putText(img, label, (x, y + 30), FONT, 3, color, 2)     #print label of detected apple on the frame
                        print("ids:"+str(class_ids))
                        print("confd:"+str(confidences))
                        print("boxes:"+str(boxes))
                        print("indexes:"+str(indexes))

                cv2.imshow("Stream", img)                                           #print the frame after detection

                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break

            except:                                                                 #used to pass excution on reciving corrupted frame
                pass
            webcam.release()
            if q.empty():
                q.put("empty")
##############################################################################################
def cord_sender(q):
    ADDR = ("192.168.233.100" ,PORT)                                        #PI IP 192.168.233.100
    client= socket.socket(socket.AF_INET,socket.SOCK_STREAM)                #create a new socket wit default settings
    
    while True:
        try:
            client.connect(ADDR)                                            #connect to the socket at ADDR
            break
        except:
            print("[COORDINATES_SERVER]ERROR:Can't connect (Re-enter IP or port),(an another server exists on port),(server didn't start)")

    print ("[COORDINATES_SERVER]connected to PI")
    
    def send(msg):                                                          #defining what to send
        message = msg.encode(FORMAT)                                        #encode the message
        msg_length = len(message)                                           #get message length
        send_length = str(msg_length).encode(FORMAT)                        #encode message length 
        send_length += b' ' * (HEADER - len(send_length))                   #prepare the message length to be send first ,"6    " if HEADER=5
        client.send(send_length)                                            #send message length
        client.send(message)                                                #send message itself
        print("[COORDINATES_SERVER]sent msg to PI:"+msg)  

    while True:                                                     
        if q.empty():                                                       #if there's no coordinates to be sent
            time.sleep(0.25)                                                #wait for 0.25 sec
            pass
        else:
            x=str(q.get())                                                  #else:send the coordinates
            send(x)
##############################################################################################
def flag_listener(qf):
    port=PORT+1
    ADDR= ("192.168.233.251" ,port)                                         #laptop IP
    #print(ADDR)
    server= socket.socket(socket.AF_INET,socket.SOCK_STREAM)                #create a new socket wit default settings
    try:
        server.bind(ADDR)                                                   #Bind the socket to address
    except:
        print("[FLAG_SERVER]ERROR:Another connection on port")          

    def handle_client(conn, addr):
        print("[FLAG_SERVER]PI "+str(addr)+" is connected")
        connected = True
        while connected:
            msg_length = conn.recv(HEADER).decode(FORMAT)                   #receive incoming message length ,assume the first message length = HEADER
            if msg_length:                                                  #if message length is defined
                msg_length = int(msg_length)                                #convert message length to int
                msg = conn.recv(msg_length).decode(FORMAT)                  #receive incoming message
                if msg == DISCONNECT_MESSAGE:                               #disconnect condition
                    connected=False
                    print("[FLAG_SERVER]PI "+str(addr)+" DISCONNECTED")
                    quit()
                else:                                                       
                    print("[FLAG_SERVER]msg recived from PI "+str(addr)+": "+msg)
                    if msg == "scan":                                       #store scan flag in flag queue
                        qf.put(msg)
        conn.close

    def start():
        server.listen()                                                          #server is listen for incoming connection on ADDR
        print("[FLAG_SERVER]server is listening on "+str(ADDR))
        while True:
            conn, addr = server.accept()                                         #accept the incoming connection 
            thread = threading.Thread(target=handle_client, args= (conn, addr))  #run flag_listener in a separate thread for the incoming connection
            thread.start()

    print("[FLAG_SERVER]server is starting")
    start()

thread1 = threading.Thread(target=process_img, args=(q,qf,))                #run process_img in a separate thread
thread2 = threading.Thread(target=cord_sender, args=(q,))                   #run cord_sender in a separate thread
thread3 = threading.Thread(target=flag_listener, args=(qf,))                #run flag_listener in a separate thread

thread1.start()
thread2.start()
thread3.start()