import telnetlib, time

class Lutron:

	def __init__(self, host, port, timeout):
		self.host = host
		self.port = int(port)
		self.timeout = int(timeout)
		
	def __del__(self):
		self.conn.close()
		
	# Connect to lighting gateway
	def connect(self):
		self.conn = telnetlib.Telnet(self.host, self.port)
	
	# Disconnect from lighting gateway
	def disconnect(self):
		self.conn.close()

	# Read data from lighting gateway
	def readData(self):
		self.lutronMsg = self.conn.read_until("\n", self.timeout)
		return self.lutronMsg
		
	# Send command to lighting gateway
	def sendCmd(self, cmd):
		fullCmd = str(cmd) + "\r\n"
#		print "Sending: %s" % fullCmd.rstrip()
		self.conn.write(fullCmd)




