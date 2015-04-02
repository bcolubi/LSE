import threading
import socket
import select
import sys
import string,cgi,time
from os import curdir, sep
#from BaseHTTPServer import BaseHTTPRequestHandler, HTTPServer


             
#s4 = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
#s4.bind((HOST4, PORT4))
#s4.listen(1)	


#try:
#	server = HTTPServer(('127.0.0.1', 8080), MyHandler)
#	print 'started httpserver...'

#except KeyboardInterrupt:
#	print '^C received, shutting down server'
#       server.socket.close()	

#data4 = 0


try:
	HOST1 = ''                
	PORT1 = 8000              
	s1 = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
	s1.bind((HOST1, PORT1))
	s1.listen(1)

	HOST2 = ''                
	PORT2 = 8001              
	s2 = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
	s2.bind((HOST2, PORT2))
	s2.listen(1)

	HOST3 = ''                
	PORT3 = 8002              
	s3 = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
	s3.bind((HOST3, PORT3))
	s3.listen(1)


	HOST4 = '127.0.0.1'                
	PORT4 = 8080
	serverSocket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
	serverSocket.bind((HOST4, PORT4))
	serverSocket.listen(1);

	inputs =[s1,s2,s3,serverSocket]
	luminosidad = 0
	temperatura = 0
	decibelios = 0

	while True:
		(read, write, exc) =  select.select(inputs,[],[])
		for s in read:
			if s == s1:
				conn, addr = s.accept()
				luminosidad = conn.recv(1024)
				print "data received from client1: "+luminosidad
				conn.sendall(luminosidad)
			elif s == s2:
				conn, addr = s.accept()
				temperatura = conn.recv(1024)
				print "data received from client2: "+temperatura
				conn.sendall(temperatura)
			elif s == s3:
				conn, addr = s.accept()
				decibelios = conn.recv(1024)
				print "data received from client3: "+decibelios
				conn.sendall(decibelios)
			elif s == serverSocket:
					print 'Ready to serve...'
					connectionSocket, addr = serverSocket.accept()
					try:	
						message = connectionSocket.recv(1024)
	    					filename = message.split()[1]
						path = "index.html"
						if (filename == '/' or filename == '/index.html'):
							f = open(curdir + sep + path)
	    						outputdata = f.read()
							connectionSocket.send('HTTP/1.1 200 OK\nContent-Type: text/html\n\n')
							#Send the content of the requested file to the client
							##for i in range(0, len(outputdata)):
							##	connectionSocket.send(outputdata[i])
							connectionSocket.send(outputdata.replace("Luminosidad:","Luminosidad:%s"%(luminosidad)).replace("Temperatura:","Temperatura:%s"%(temperatura)).replace("Decibelios:","Decibelios: %s"%(decibelios)))
							connectionSocket.close()
						else:
						    	connectionSocket.send('HTTP/1.1 404 File not found\n') 
							connectionSocket.close()	
					except IOError:
						connectionSocket.send('HTTP/1.1 404 File not found\n') 
						connectionSocket.shutdown(1)						
						connectionSocket.close()
				
			else: 
				print "webserver requesting but no detected... "
				continue

except (KeyboardInterrupt, SystemExit):
	print '^C received, shutting down server'	
	serverSocket.shutdown(1)
	serverSocket.close()





