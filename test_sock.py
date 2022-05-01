import socket
import struct
import sys

s = socket.socket()
s.connect(("127.0.0.1", 8756))
cmd = sys.argv[1]
val = sys.argv[2]
#print('cmd %s'%(cmd))
data = bytearray()
data.append(0xBA)
data.append(0xDC)
data.append(int(cmd))  #0x1: threshold; 0x2: record; 0x3: source uri
#data.append(0x1)  #0x1: threshold; 0x2: record; 0x3: source uri
data.append(0x77) #0x72: read; 0x77: write
data.append(int(val)) #0x72: read; 0x77: write
#uri='www.googlw.com'
#data += bytearray(uri.encode()) #0x72: read; 0x77: write
#threshold = 0.05
#data += struct.pack('f', threshold)


print(data)
s.send(data)
reply = s.recv(8)
print(reply)
print(struct.unpack('<d', reply)[0])
#print(reply.decode("utf-8"))
s.close()

