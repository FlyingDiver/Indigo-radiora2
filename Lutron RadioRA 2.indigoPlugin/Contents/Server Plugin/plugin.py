#! /usr/bin/env python
# -*- coding: utf-8 -*-
####################
# Lutron RadioRA 2 server plugin
#
# By Jim Lombardo jim@jimandnoreen.com
# Use as you see fit.  Please share your improvements.
#
# More info and instructions for this plugin at http://jimandnoreen.com/?p=96
#
#
# This plugin is for Lutron RadioRA 2 and Caséta systems only and is NOT compatible with
# the classic RadioRA command set.
#
# If you have an older classic RadioRA system, please use this plugin instead:
#
# http://www.whizzosoftware.com/forums/blog/1/entry-50-indigo-and-lutron-radiora/
#
#
# Changelog
#
# 1.0.0 initial release
# 1.0.1 fixed handling of output flash commands and added thermostat setpoint handling
# 1.1.0 improved thermostat ui and added keypad support (code generously contributed by Sylvain B.)
# 1.1.1 addressed undocumented "action 29" protocol change in RadioRA 2 7.2 (thanks to Bill L. and Sylvain B.)
# 1.1.2 fixed sending of keypad Press and Release serial commands
# 1.2.0 added support for motion sensors (thanks to Tony W.)
# 1.2.1 added notes field for all devices, populate address with integration id
# 1.2.2 added option to query all devices at startup and menu option to toggle debug mode
# 1.2.3 fixed bug with setting LED status and added option to follow LED state instead of corresponding button press (thanks Swancoat)
# 1.2.4 improved keypad configuration dialog
# 1.2.5 ignore actions that are not explicitly defined like undocumented "action 29" and "action 30" (thanks FlyingDiver)
# 1.2.6 added explicit support for motorized shades, CCO and CCI devices (thanks rapamatic!!) and improved device/output logging
# 2.0.0 added Caséta support by mathys and IP connectivity contributed by Sb08 and vic13.  Added menu option to query all devices
# 2.0.3 added Pico device type.  Changed CCI device type from relay to sensor.  Restrict query all devices to this plugin's devices.
# 2.1.0 added Group and TimeClock events.  Added BrightenBy and DimBy command support.  Added GitHubPluginUpdater support.

from __future__ import with_statement

import functools
import os, socket
import serial
import sys
import threading
import time
import indigo
import string

from telnetlib import select
from lutron import Lutron
from ghpu import GitHubPluginUpdater


RA_PHANTOM_BUTTON = "ra2PhantomButton"
RA_DIMMER = "ra2Dimmer"
RA_SWITCH = "ra2Switch"
RA_KEYPAD = "ra2Keypad"
RA_FAN = "ra2Fan"
RA_THERMO = "ra2Thermo"
RA_SENSOR = "ra2Sensor"
RA_CCO = "ra2CCO"
RA_CCI = "ra2CCI"
RA_SHADE = "ra2MotorizedShade"
RA_PICO = "ra2Pico"
PROP_BUTTON = "button"
PROP_ZONE = "zone"
PROP_SWITCH = "switch"
PROP_KEYPAD = "keypad"
PROP_KEYPADBUT = "keypadButton"
PROP_FAN = "fan"
PROP_THERMO = "thermo"
PROP_SENSOR = "sensor"
PROP_NOTES = "notes"
PROP_AREA = "area"
PROP_KEYPADBUT_DISPLAY_LED_STATE = "keypadButtonDisplayLEDState"
PROP_CCO_INTEGRATION_ID = "ccoIntegrationID"
PROP_CCO_TYPE = "ccoType"
PROP_CCI_INTEGRATION_ID = "cciIntegrationID"
PROP_COMPONENT = "component"
PROP_SUPPORTS_STATUS_REQUEST = "SupportsStatusRequest"
PROP_SHADE = "shade"
PROP_PICO_INTEGRATION_ID = "picoIntegrationID"
PROP_PICOBUTTON = "picoButton"

########################################
class Plugin(indigo.PluginBase):
########################################
	def __init__(self, pluginId, pluginDisplayName, pluginVersion, pluginPrefs):
		indigo.PluginBase.__init__(self, pluginId, pluginDisplayName, pluginVersion, pluginPrefs)

		if 'debugEnabled' in pluginPrefs:
			self.debug = pluginPrefs['debugEnabled']
		else:
			self.debug = False

		if 'queryAtStartup' in pluginPrefs:
			self.queryAtStartup = pluginPrefs['queryAtStartup']
		else:
			self.queryAtStartup = False
			
		self.conn = {}
		self.command = ''
		self.phantomButtons = {}
		self.keypads = {}
		self.zones = {}
		self.switches = {}
		self.lastBrightness = {}
		self.fans = {}
		self.thermos = {}
		self.sensors = {}
		self.ccis = {}
		self.ccos = {}
		self.shades = {}
		self.picos = {}
		self.runstartup = False
		self.ePanel = Lutron("127.0.0.1", "23", 1)
		self.IP = False		# Default to serial I/O, not IP -vic13
		self.caseta = False	# Default to RadioRA 2, not Caséta -vic13
		self.portEnabled = False
		self.triggers = { }

		
	def __del__(self):
		indigo.PluginBase.__del__(self)

	def startup(self):
		self.debugLog(u"startup called")
		
		self.updater = GitHubPluginUpdater(self)
		self.updater.checkForUpdate()
		self.updateFrequency = self.pluginPrefs.get('updateFrequency', 24)
		if self.updateFrequency > 0:
			self.next_update_check = time.time() + float(self.updateFrequency) * 60.0 * 60.0

		# Call IP startup routine, but only run them if the IP box is checked -vic13
		try:
			self.IPstartup()
			self.runstartup = False
		except KeyError:
			self.errorLog(u"Plugin not configured. Delaying startup until configuration is saved")

		# When not doing IP communications, run original serial setup routines	-vic13		
		try:
			self.serialStartup()
			self.runstartup = False
		except KeyError:
			self.errorLog(u"Plugin not configured. Delaying startup until configuration is saved")

	def shutdown(self):
		self.debugLog(u"shutdown called")


	####################

	def triggerStartProcessing(self, trigger):
		self.debugLog(u"Adding Trigger %s (%d) - %s" % (trigger.name, trigger.id, trigger.pluginTypeId))
		assert trigger.id not in self.triggers
		self.triggers[trigger.id] = trigger
 
	def triggerStopProcessing(self, trigger):
		self.debugLog(u"Removing Trigger %s (%d)" % (trigger.name, trigger.id))
		assert trigger.id in self.triggers
		del self.triggers[trigger.id] 
		
	def clockTriggerCheck(self, info):

		for triggerId, trigger in sorted(self.triggers.iteritems()):
			type = trigger.pluginTypeId
			eventNumber = trigger.pluginProps["eventNumber"]
			self.debugLog(u"Checking Trigger %s (%s), Type: %s, Event: %s" % (trigger.name, trigger.id, type, eventNumber))
			
			if "timeClockEvent" != type:
				self.debugLog(u"\tSkipping Trigger %s (%s), wrong type: %s" % (trigger.name, trigger.id, type))
				return
				
			if eventNumber != info:
				self.debugLog(u"\tSkipping Trigger %s (%s), wrong event: %s" % (trigger.name, trigger.id, info))
				return
				
			self.debugLog(u"\tExecuting Trigger %s (%s)" % (trigger.name, trigger.id))
			indigo.trigger.execute(trigger)
			
	def groupTriggerCheck(self, info):

		for triggerId, trigger in sorted(self.triggers.iteritems()):
			type = trigger.pluginTypeId
			groupNumber = trigger.pluginProps["groupNumber"]
			self.debugLog(u"Checking Trigger %s (%s), Type: %s, Group: %s" % (trigger.name, trigger.id, type, groupNumber))
			
			if "groupEvent" != type:
				self.debugLog(u"\tSkipping Trigger %s (%s), wrong type: %s" % (trigger.name, trigger.id, type))
				return
				
			if groupNumber != info:
				self.debugLog(u"\tSkipping Trigger %s (%s), wrong group: %s" % (trigger.name, trigger.id, info))
				return
				
			self.debugLog(u"\tExecuting Trigger %s (%s)" % (trigger.name, trigger.id))
			indigo.trigger.execute(trigger)
			
	####################
	
			

	def	update_device_property ( self, dev, propertyname, new_value = ""):
		newProps = dev.pluginProps
		newProps.update ( {propertyname : new_value} )
		dev.replacePluginPropsOnServer(newProps)
		return None


	def deviceStartComm(self, dev):
		if dev.deviceTypeId == RA_PHANTOM_BUTTON:
			self.phantomButtons[dev.pluginProps[PROP_BUTTON]] = dev
			self.update_device_property ( dev, "address", new_value = dev.pluginProps[PROP_BUTTON] )
			self.debugLog(u"Watching phantom button: " + dev.pluginProps[PROP_BUTTON])
		elif dev.deviceTypeId == RA_DIMMER:
			self.zones[dev.pluginProps[PROP_ZONE]] = dev
			self.update_device_property ( dev, "address", new_value = dev.pluginProps[PROP_ZONE] )
			self.debugLog(u"Watching dimmer: " + dev.pluginProps[PROP_ZONE])
		elif dev.deviceTypeId == RA_SHADE:
			self.shades[dev.pluginProps[PROP_SHADE]] = dev
			self.update_device_property ( dev, "address", new_value = dev.pluginProps[PROP_SHADE] )
			dev.updateStateImageOnServer( indigo.kStateImageSel.None )
			self.debugLog(u"Watching shade: " + dev.pluginProps[PROP_SHADE])
		elif dev.deviceTypeId == RA_SWITCH:
			self.switches[dev.pluginProps[PROP_SWITCH]] = dev
			self.update_device_property ( dev, "address", new_value = dev.pluginProps[PROP_SWITCH] )
			self.debugLog(u"Watching switch: " + dev.pluginProps[PROP_SWITCH])
		elif dev.deviceTypeId == RA_FAN:
			self.fans[dev.pluginProps[PROP_FAN]] = dev
			self.update_device_property ( dev, "address", new_value = dev.pluginProps[PROP_FAN] )
			self.debugLog(u"Watching fan: " + dev.pluginProps[PROP_FAN])
		elif dev.deviceTypeId == RA_THERMO:
			self.thermos[dev.pluginProps[PROP_THERMO]] = dev
			self.update_device_property ( dev, "address", new_value = dev.pluginProps[PROP_THERMO] )
			self.debugLog(u"Watching thermostat: " + dev.pluginProps[PROP_THERMO])
		elif dev.deviceTypeId == RA_KEYPAD:
			self.keypads[dev.pluginProps[PROP_KEYPAD]+dev.pluginProps[PROP_KEYPADBUT]] = dev
			self.update_device_property ( dev, "address", new_value = dev.pluginProps[PROP_KEYPAD] + "." + dev.pluginProps[PROP_KEYPADBUT])
			if dev.pluginProps[PROP_KEYPADBUT] > 80:
				self.update_device_property ( dev, "keypadButtonDisplayLEDState", new_value = dev.pluginProps[PROP_KEYPADBUT_DISPLAY_LED_STATE] )
				self.debugLog(u"Watching keypad: " + dev.pluginProps[PROP_KEYPAD] + " LED: " + dev.pluginProps[PROP_KEYPADBUT])
			else:
				self.debugLog(u"Watching keypad: " + dev.pluginProps[PROP_KEYPAD] + " button: " + dev.pluginProps[PROP_KEYPADBUT])
		elif dev.deviceTypeId == RA_SENSOR:
			self.sensors[dev.pluginProps[PROP_SENSOR]] = dev
			self.update_device_property ( dev, "address", new_value = dev.pluginProps[PROP_SENSOR] )
			self.debugLog(u"Watching sensor: " + dev.pluginProps[PROP_SENSOR])
		elif dev.deviceTypeId == RA_CCI:
			self.ccis[dev.pluginProps[PROP_CCI_INTEGRATION_ID]+dev.pluginProps[PROP_COMPONENT]] = dev
			self.update_device_property ( dev, "address", new_value = dev.pluginProps[PROP_CCI_INTEGRATION_ID] + "." + dev.pluginProps[PROP_COMPONENT])
			self.debugLog(u"Watching CCI: " + dev.pluginProps[PROP_CCI_INTEGRATION_ID] + " input: " + dev.pluginProps[PROP_COMPONENT])
		elif dev.deviceTypeId == RA_CCO:
			self.ccos[dev.pluginProps[PROP_CCO_INTEGRATION_ID]] = dev
			self.update_device_property ( dev, "address", new_value = dev.pluginProps[PROP_CCO_INTEGRATION_ID] )
			ccoType = dev.pluginProps[PROP_CCO_TYPE]
			if ccoType == "momentary":
				dev.updateStateOnServer("onOffState", False)
			# To do - set SupportsStatusRequest to true if it is a sustained contact CCO
			#         haven't figured out a way to do that without hanging the UI when a new CCO is added
			self.debugLog(u"Watching CCO: " + dev.pluginProps[PROP_CCO_INTEGRATION_ID])
		elif dev.deviceTypeId == RA_PICO:
			self.picos[dev.pluginProps[PROP_PICO_INTEGRATION_ID]+dev.pluginProps[PROP_PICOBUTTON]] = dev
			self.update_device_property ( dev, "address", new_value = dev.pluginProps[PROP_PICO_INTEGRATION_ID] + "." + dev.pluginProps[PROP_PICOBUTTON])
			self.debugLog(u"Watching Pico: " + dev.pluginProps[PROP_PICO_INTEGRATION_ID] + " button: " + dev.pluginProps[PROP_PICOBUTTON])

	def deviceStopComm(self, dev):
		if dev.deviceTypeId == RA_PHANTOM_BUTTON:
			del self.phantomButtons[dev.pluginProps[PROP_BUTTON]]
			self.debugLog(u"Deleted phantom button: " + dev.pluginProps[PROP_BUTTON])
		elif dev.deviceTypeId == RA_DIMMER:
			del self.zones[dev.pluginProps[PROP_ZONE]]
			self.debugLog(u"Deleted dimmer: " + dev.pluginProps[PROP_ZONE])
		elif dev.deviceTypeId == RA_SHADE:
			del self.shades[dev.pluginProps[PROP_SHADE]]
			self.debugLog(u"Deleted shade: " + dev.pluginProps[PROP_SHADE])
		elif dev.deviceTypeId == RA_SWITCH:
			del self.switches[dev.pluginProps[PROP_SWITCH]]
			self.debugLog(u"Deleted switch: " + dev.pluginProps[PROP_SWITCH])
		elif dev.deviceTypeId == RA_KEYPAD:
			del self.keypads[dev.pluginProps[PROP_KEYPAD]+dev.pluginProps[PROP_KEYPADBUT]] 
			self.debugLog(u"Deleted keypad: " + dev.pluginProps[PROP_KEYPAD]+dev.pluginProps[PROP_KEYPADBUT])
		elif dev.deviceTypeId == RA_FAN:
			del self.fans[dev.pluginProps[PROP_FAN]]
			self.debugLog(u"Deleted fan: " + dev.pluginProps[PROP_FAN])
		elif dev.deviceTypeId == RA_THERMO:
			del self.thermos[dev.pluginProps[PROP_THERMO]]
			self.debugLog(u"Deleted thermostat: " + dev.pluginProps[PROP_THERMO])
		elif dev.deviceTypeId == RA_SENSOR:
			del self.sensors[dev.pluginProps[PROP_SENSOR]]
			self.debugLog(u"Deleted sensor: " + dev.pluginProps[PROP_SENSOR])
		elif dev.deviceTypeId == RA_CCI:
			del self.ccis[dev.pluginProps[PROP_CCI_INTEGRATION_ID]+dev.pluginProps[PROP_COMPONENT]]
			self.debugLog(u"Deleted CCI: " + dev.pluginProps[PROP_CCI_INTEGRATION_ID]+dev.pluginProps[PROP_COMPONENT])
		elif dev.deviceTypeId == RA_CCO:
			del self.ccos[dev.pluginProps[PROP_CCO_INTEGRATION_ID]]
			self.debugLog(u"Deleted CCO: " + dev.pluginProps[PROP_CCO_INTEGRATION_ID])
		elif dev.deviceTypeId == RA_PICO:
			del self.picos[dev.pluginProps[PROP_PICO_INTEGRATION_ID]+dev.pluginProps[PROP_PICOBUTTON]]
			self.debugLog(u"Deleted Pico: " + dev.pluginProps[PROP_PICO_INTEGRATION_ID]+dev.pluginProps[PROP_PICOBUTTON])

	def runConcurrentThread(self):
	# Select threads based on IP or serial communications - vic13
		if self.IP:
			self.debugLog(u"Starting IP monitor thread")
			try:
				while True:
					self.sleep(.1)

					# Plugin Update check
				
					if self.updateFrequency > 0:
						if time.time() > self.next_update_check:
							self.updater.checkForUpdate()
							self.next_update_check = time.time() + float(self.pluginPrefs['updateFrequency']) * 60.0 * 60.0
					
					try:
						if self.runstartup:
							self.debugLog(u"Calling IP Startup")
							self.IPstartup()
							self.runstartup = False

						self.dispatchMsg(self.ePanel.readData())
#						else:
#							pass
					except EOFError, e:
						self.errorLog(u"EOFError: %s" % e.message)
						if ('telnet connection closed' in e.message):
							self.runstartup = True
							self.sleep(10)
					except AttributeError, e:
						self.debugLog(u"AttributeError: %s" % e.message)
					except select.error, e:
						self.debugLog(u"Disconnected while listening: %s" % e.message)
	#				except:
	#					self.errorLog(u"Unknown Error: %s" % sys.exc_info()[0])
			except self.StopThread:
				pass
		else:	# Added by vic13


			
			self.debugLog(u"Starting serial monitor thread")

			while not self.portEnabled:
				self.sleep(.1)

			try:
				while True:
					if self.runstartup:
						self.debugLog(u"Calling Serial Startup")
						self.serialStartup()
						self.runstartup = False

					# Plugin Update check
				
					if self.updateFrequency > 0:
						if time.time() > self.next_update_check:
							self.updater.checkForUpdate()
							self.next_update_check = time.time() + float(self.pluginPrefs['updateFrequency']) * 60.0 * 60.0

					s = self.conn.read()
					if self.stopThread:
						self.debugLog(u"Ending serial monitor thread")
						return
					else:
						if len(s) > 0:
							# RadioRA 2 messages are always terminated with CRLF
							if s == '\r':
								self._processCommand(self.command)
								self.command = ''
							else:
								self.command += s
			except self.StopThread:
				pass

	def serialStartup(self):
		self.IP = self.pluginPrefs["IP"]
		if not self.IP:
			try:
				if 'debugEnabled' in self.pluginPrefs:
					self.debug = self.pluginPrefs['debugEnabled']
					self.debugLog(u"Debug logging enabled.")
				else:
					self.debug = False
			except KeyError:
				self.errorLog(u"Plugin not yet configured.\nPlease save the configuration then reload the plugin.\nThis should only happen the first time you run the plugin\nor if you delete the preferences file.")
				return

			self.portEnabled = False
			
			serialUrl = self.getSerialPortUrl(self.pluginPrefs, u"devicePort")
			self.debugLog(u"Serial Port URL is: " + serialUrl)
			
			self.conn = self.openSerial(u"Lutron RadioRA", serialUrl, 9600, stopbits=1, timeout=2, writeTimeout=1)
			if self.conn is None:
				indigo.server.log(u"Failed to open serial port")
				return
			
			self.portEnabled = True
				
			# Disable main repeater terminal prompt
			self._sendCommand("#MONITORING,12,2")
					
			# Enable main repeater HVAC monitoring
			self._sendCommand("#MONITORING,17,1")
						
			# Enable main repeater monitoring param 18
			# (undocumented but seems to be enabled by default for ethernet connections)
			self._sendCommand("#MONITORING,18,1")
			
			if self.queryAtStartup:
				self.queryAllDevices()

	def IPstartup(self):
		# Check to see if we are using IP or serial communications. - vic13
		self.IP = self.pluginPrefs["IP"]
		
		# Only start IP routines if IP box is checked	- vic13
		if self.IP:
			try:
				if 'debugEnabled' in self.pluginPrefs:
					self.debug = self.pluginPrefs['debugEnabled']
					self.debugLog(u"Debug logging enabled.")
				else:
					self.debug = False
			except KeyError:
				self.errorLog(u"Plugin not yet configured.\nPlease save the configuration then reload the plugin.\nThis should only happen the first time you run the plugin\nor if you delete the preferences file.")
				return
			
			if 1 == 1:
				self.debugLog(u"Creating instance of class Lutron.")
				host = self.pluginPrefs["ip_address"]
				port = self.pluginPrefs["ip_port"]
				self.ePanel = Lutron(host, port, 1)
				self.debugLog(u"Lutron class instance %s created." % repr(self.ePanel))
				self.debugLog(u"Initiating connection to Lutron gateway.")

				try:
					self.ePanel.connect()
					self.debugLog(u"Connecting...")
				
					a = self.ePanel.readData()
					self.debugLog(u"%s" % a)
				
					if 'login' in a:
						self.ePanel.sendCmd(str(self.pluginPrefs["ip_username"]))
						self.debugLog(u"Connected to panel, sending username.")
					
						a = self.ePanel.readData()
						self.debugLog(u"%s" % a)
						if 'password' in a:
							self.ePanel.sendCmd(str(self.pluginPrefs["ip_password"]))
							self.debugLog(u"sending password.")
						else:
							self.debugLog(u"password failure.")
					else:
						self.debugLog(u"username failure.")
#					indigo.devices[self.panelId].updateStateOnServer("conn_state", "On")
					self.debugLog(u"end of connection process.")
				
				

				except socket.error, e:
					self.errorLog(u"Unable to connect to Lutron gateway. %s" % e.message)
#					indigo.devices[self.panelId].updateStateOnServer("conn_state", "Off")


				if self.queryAtStartup:
						self.queryAllDevices()


#########################################
# Poll registered devices for status
#########################################
	def queryAllDevices(self):
		for dev in indigo.devices.iter("self"):
			indigo.device.statusRequest(dev)


# plugin configuration validation
	def validatePrefsConfigUi(self, valuesDict):
		errDict = indigo.Dict()

		badAddr = "Please use either an IP address (i.e. 1.2.3.4) or a fully qualified host name (i.e. lutron.domain.com)"
		newaddr = str("%s:%s" % (valuesDict["ip_address"], valuesDict["ip_port"]))

		if valuesDict["debugEnabled"]:
			self.debug = True
		else:
			self.debug = False
        
		self.IP = valuesDict["IP"]

		if 1==1:
			if valuesDict["ip_address"].count('.') >= 3:
				ipOK = True
			else:
				ipOK = False

			try:
				if ipOK:
					rtn = True
				else:
					errDict["ip_address"] = badAddr
					rtn = (False, valuesDict, errDict)
			except AttributeError:
				rtn = (True, valuesDict)
		try:
			if valuesDict["configDone"]:
				self.runstartup = False
			else:
				if ipOK and rtn:
					self.debugLog(u"Setting configDone to True")
					valuesDict["configDone"] = True
					self.debugLog(u"Setting flag to run startup")
					self.runstartup = True
		except KeyError:
			if ipOK and rtn:
				self.debugLog(u"Setting configDone to True")
				valuesDict["configDone"] = True
				self.debugLog(u"Setting flag to run startup")
				self.runstartup = True
		self.IP = valuesDict["IP"]
		self.debugLog(u"%s, %s, %s" % (str(rtn), str(ipOK), str(self.IP)))
		return rtn

	

	def dispatchMsg(self, msg):
		try:
			msg = msg.rstrip()
			if len(msg) > 0:
#				self.debugLog(u"Message received: %s" % msg)
				# RadioRA 2 messages are always terminated with CRLF
				if "~OUTPUT" in msg:
					self._cmdOutputChange(msg)
				elif "~DEVICE" in msg:
					self._cmdDeviceChange(msg)
				elif "~HVAC" in msg:
					self._cmdHvacChange(msg)
				elif "~TIMECLOCK" in msg:
					self._cmdTimeClock(msg)
				elif "~MONITORING" in msg:
					self.debugLog(u"Main repeater serial interface configured" + msg)
				elif "~GROUP" in msg:
					self._cmdGroup(msg)
				elif 'GNET' in msg:
					#command prompt is ready					
					self.debugLog(u"Command prompt received. Device is ready.")
				elif msg != "!":
					self.errorLog(u" Unrecognized command: " + msg)
		except self.StopThread:
			pass

	def _sendCommand(self, cmd):
        # Choose IP or serial routines - vic13
		# Need to add the \n for Caseta here in the IP section
		if self.IP:
			self.debugLog(u"Sending network command: " + cmd)
			# Caseta needs "\n" appended
			if self.caseta:
				cmd = cmd + "\n"
			self.ePanel.sendCmd(cmd)
		else:
			self.debugLog(u"Sending serial command: " + cmd)	
			self.conn.write(cmd + "\r") # \r needed for serial -vic13

	# IP communications use dispatchMsg() instead. -vic13
	def _processCommand(self, cmd):
		if "~OUTPUT" in cmd:
			self._cmdOutputChange(cmd)
		elif "~DEVICE" in cmd:
			self._cmdDeviceChange(cmd)
		elif "~HVAC" in cmd:
			self._cmdHvacChange(cmd)
		elif "~TIMECLOCK" in cmd:
			self._cmdTimeClock(cmd)
		elif "~GROUP" in cmd:
			self._cmdGroup(cmd)
		elif "~MONITORING" in cmd:
			self.debugLog(u"Main repeater serial interface configured" + cmd)
		elif cmd != "!":
			self.debugLog(u"Unrecognized command: " + cmd)


	def _cmdOutputChange(self,cmd):
		self.debugLog(u"Received an Output message: " + cmd)
		cmdArray = cmd.split(',')
		id = cmdArray[1]
		action = cmdArray[2]
		if action == '1':  # set level
			level = cmdArray[3]

			# something else to consider for future enhancements
			# fade = cmdArray[4]
			# delay = cmdArray[5]
			if id in self.zones:
				zone = self.zones[id]
				if int(float(level)) == 0:
					zone.updateStateOnServer("onOffState", False)
				else:
					zone.updateStateOnServer("onOffState", True)
					zone.updateStateOnServer("brightnessLevel", int(float(level)))
				indigo.server.log(u"Received: Dimmer " + zone.name + " level set to " + str(level))
			elif id in self.shades:
				shade = self.shades[id]
				if int(float(level)) == 0:
					shade.updateStateOnServer("onOffState", False)
				else:
					shade.updateStateOnServer("onOffState", True)
					shade.updateStateOnServer("brightnessLevel", int(float(level)))
				indigo.server.log(u"Received: Shade " + shade.name + " opening set to " + str(level))
			elif id in self.switches:
				switch = self.switches[id]
				if int(float(level)) == 0:
					switch.updateStateOnServer("onOffState", False)
					indigo.server.log(u"Received: Switch %s %s" % (switch.name, "turned Off"))
				else:
					switch.updateStateOnServer("onOffState", True)
					indigo.server.log(u"Received: Switch %s %s" % (switch.name, "turned On"))
			elif id in self.ccos:
				cco = self.ccos[id]
				ccoType = cco.pluginProps[PROP_CCO_TYPE]
				if ccoType == "sustained":
					if int(float(level)) == 0:
					 cco.updateStateOnServer("onOffState", False)
					else:
					 cco.updateStateOnServer("onOffState", True)
				if level == '0.00':
					indigo.server.log(u"Received: CCO %s %s" % (cco.name, "Opened"))
				else:
					indigo.server.log(u"Received: CCO %s %s" % (cco.name, "Closed"))
			elif id in self.fans:
				fan = self.fans[id]
				if level == '0.00':
					fan.updateStateOnServer("onOffState", False)
				else:
					fan.updateStateOnServer("onOffState", True)
					if level == '25.10':
						fan.updateStateOnServer("speedIndex", 1)
					elif level == '50.20':
						fan.updateStateOnServer("speedIndex", 2)
					elif level == '75.30':
						fan.updateStateOnServer("speedIndex", 2)
					else:
						fan.updateStateOnServer("speedIndex", 3)
				indigo.server.log(u"Received: Fan " + fan.name + " speed set to " + str(level))
				return
		elif action == '2':  # start raising
			return
		elif action == '3':  # start lowering
			return
		elif action == '4':  # stop raising/lowering
			return
		elif action == '5':  # start flash
			return
		elif action == '6':  # pulse
			return
		elif action == '29':  # Lutron firmware 7.5 added an undocumented 29 action code; ignore for now
			return
		elif action == '30':  # Lutron firmware ??? added an undocumented 30 action code; ignore for now
			return
		return
		
	def _cmdDeviceChange(self,cmd):
		self.debugLog(u"Received a Device message: " + cmd)

		if self.IP:
			cmd = cmd.rstrip() # IP strings are terminated with \n -JL

		cmdArray = cmd.split(',')
		id = cmdArray[2]
		action = cmdArray[3]
		if action == '2': # this is a motion sensor
			if cmdArray[4] == '3':
				status = '1'
			elif cmdArray[4] == '4':
				status = '0'
		elif action == '3':
			status = '1'
		elif action == '4':
			status = '0'
		else:
			status = cmdArray[4]

		if id in self.phantomButtons:
			self.debugLog(u"Received a phantom button status message: " + cmd)
			but = self.phantomButtons[id]
			if status == '0':
				but.updateStateOnServer("onOffState", False)
			elif status == '1':
				but.updateStateOnServer("onOffState", True)

		id = cmdArray[1]
		button = cmdArray[2]
		keypadid = id+button

		if keypadid in self.keypads:
			self.debugLog(u"Received a keypad button/LED status message: " + cmd)
			dev = self.keypads[keypadid]
			keypad = self.keypads[keypadid]
			if status == '0':
				keypad.updateStateOnServer("onOffState", False)
			elif status == '1':
				keypad.updateStateOnServer("onOffState", True)
		
			if dev.pluginProps[PROP_KEYPADBUT_DISPLAY_LED_STATE]: # Also display this LED state on its corresponding button
				self.debugLog(keypadid)
				keypadid = keypadid[0:len(keypadid)-2] + keypadid[len(keypadid)-1] # Convert LED ID to button ID
				self.debugLog(keypadid)
				if keypadid in self.keypads:
					keypad = self.keypads[keypadid]
					self.debugLog(u"Updating button status with state of LED for keypadID " + keypadid)
					if int(status) == 0:
						keypad.updateStateOnServer("onOffState", False)
					elif int(status) == 1:
						keypad.updateStateOnServer("onOffState", True)
						self.debugLog(u"Set status to True on Server.")
				else:
					indigo.server.log("WARNING: Invalid ID (%s) specified for LED.  Must be in range 81-87.  Please correct and reload the plugin." % keypadid, isError=True)
					self.debugLog(keypadid)


		if keypadid in self.picos:
			self.debugLog(u"Received a pico button status message: " + cmd)
			but = self.picos[keypadid]
			if status == '0':
				but.updateStateOnServer("onOffState", False)
			elif status == '1':
				but.updateStateOnServer("onOffState", True)

					
		if keypadid in self.ccis:
			self.debugLog(u"Received a CCI status message: " + cmd)
			cci = self.ccis[keypadid]
			if status == '0':
				cci.updateStateOnServer("onOffState", False)
				indigo.server.log(u"Received: CCI %s %s" % (cci.name, "Opened"))
			elif status == '1':
				cci.updateStateOnServer("onOffState", True)
				indigo.server.log(u"Received: CCI %s %s" % (cci.name, "Closed"))

		if id in self.sensors:
			self.debugLog(u"Received a sensor status message: " + cmd)
			but = self.sensors[id]
			# self.debugLog(u"Variable But: " + but)
			if status == '0':
				but.updateStateOnServer("onOffState", False)
				indigo.server.log(u"Received: Motion Sensor %s %s" % (but.name, "vacancy detected"))
			elif status == '1':
				but.updateStateOnServer("onOffState", True)
				indigo.server.log(u"Received: Motion Sensor %s %s" % (but.name, "motion detected"))

	# IP comm has not yet been tested with _cmdHvacChange().  Currently left as is -vic13
	def _cmdHvacChange(self,cmd):
		self.debugLog(u"Received an HVAC message: " + cmd)
		cmdArray = cmd.split(',')
		id = cmdArray[1]
		action = cmdArray[2]
		if id in self.thermos:
			thermo = self.thermos[id]
			if action == '1':
				temperature = cmdArray[3]
				thermo.updateStateOnServer("temperatureInput1", float(temperature))
			elif action == '2':
				heatSetpoint = cmdArray[3]
				coolSetpoint = cmdArray[4]
				thermo.updateStateOnServer("setpointHeat", float(heatSetpoint))
				thermo.updateStateOnServer("setpointCool", float(coolSetpoint))
			elif action == '3':
				mode = cmdArray[3] #1=off, 2=heat, 3=cool, 4=auto, 5=em. heat
				if mode == '1':
					thermo.updateStateOnServer("hvacOperationMode", indigo.kHvacMode.Off)
				elif mode == '2':
					thermo.updateStateOnServer("hvacOperationMode", indigo.kHvacMode.Heat)
				elif mode == '3':
					thermo.updateStateOnServer("hvacOperationMode", indigo.kHvacMode.Cool)
				elif mode == '4':
					thermo.updateStateOnServer("hvacOperationMode", indigo.kHvacMode.HeatCool)
			elif action == '4':
				fanmode = cmdArray[3]
				if fanmode == '1':
					thermo.updateStateOnServer("hvacFanMode", indigo.kFanMode.Auto)
				elif fanmode == '2':
					thermo.updateStateOnServer("hvacFanMode", indigo.kFanMode.AlwaysOn)

	def _cmdTimeClock(self,cmd):
		self.debugLog(u"Received a TimeClock message: " + cmd)
		cmdArray = cmd.split(',')
		id = cmdArray[1]
		action = cmdArray[2]
		event = cmdArray[3]
		self.clockTriggerCheck(event)

	def _cmdGroup(self,cmd):
		self.debugLog(u"Received a Group message  " + cmd)
		cmdArray = cmd.split(',')
		id = cmdArray[1]
		action = cmdArray[2]
		status = cmdArray[3]
		self.clockTriggerCheck(status)


	##########################################
	# Toggle debug without restarting plugin
	##########################################
	def toggleDebugEnabled(self):
		self.debug = not self.debug
		self.pluginPrefs["debugEnabled"] = self.debug
		indigo.server.log("Debug set to " + str(self.debug) + " by user")

	def checkForUpdates(self):
		self.updater.checkForUpdate()

	def updatePlugin(self):
		self.updater.update()

	def forceUpdate(self):
		self.updater.update(currentVersion='0.0.0')
			


	########################################
	# Relay / Dimmer / /Shade / CCO / CCI Action callback
	########################################
	def actionControlDimmerRelay(self, action, dev):
		# Original change by Ramias.  \r is added only for serial I/O.  We use sendCmd()
		#
		sendCmd = ""
	
		###### TURN ON ######
		if action.deviceAction == indigo.kDeviceAction.TurnOn:
			if dev.deviceTypeId == RA_PHANTOM_BUTTON:
				button = dev.pluginProps[PROP_BUTTON]
				sendCmd = ("#DEVICE,1," + str(int(button)-100) + ",3") # Press button
			elif dev.deviceTypeId == RA_PICO:
				pico = dev.pluginProps[PROP_PICO_INTEGRATION_ID]
				button = dev.pluginProps[PROP_PICOBUTTON]
				sendCmd = ("#DEVICE," + pico + "," + button + ",3") # Press button
			elif dev.deviceTypeId == RA_KEYPAD:
				keypad = dev.pluginProps[PROP_KEYPAD]
				keypadButton = dev.pluginProps[PROP_KEYPADBUT]
				if (int(keypadButton) > 80):
					sendCmd = ("#DEVICE," + keypad + "," + str(int(keypadButton)) + ",9,1") # Turn on an LED
				else:
					sendCmd = ("#DEVICE," + keypad + "," + str(int(keypadButton)) + ",3") # Press button
			elif dev.deviceTypeId == RA_DIMMER:
				zone = dev.pluginProps[PROP_ZONE]
				sendCmd = ("#OUTPUT," + zone + ",1,100")
				self.lastBrightness[zone] = 100
			elif dev.deviceTypeId == RA_SHADE:
				shade = dev.pluginProps[PROP_SHADE]
				sendCmd = ("#OUTPUT," + shade + ",1,100")
			elif dev.deviceTypeId == RA_SWITCH:
				switch = dev.pluginProps[PROP_SWITCH]
				sendCmd = ("#OUTPUT," + switch + ",1,100")
			elif dev.deviceTypeId == RA_CCI:
				self.debugLog(u"it is a cci")
				cci = dev.pluginProps[PROP_CCI_INTEGRATION_ID]
				component = dev.pluginProps[PROP_COMPONENT]
				sendCmd = ("#DEVICE," + cci +"," + str(int(component)) + ",3")
			elif dev.deviceTypeId == RA_CCO:
				cco = dev.pluginProps[PROP_CCO_INTEGRATION_ID]
				ccoType = dev.pluginProps[PROP_CCO_TYPE]
				if ccoType == "momentary":
					sendCmd = ("#OUTPUT," + cco + ",6")
					sendCmd = ("#OUTPUT," + cco + ",1,1")
				else:
					sendCmd = ("#OUTPUT," + cco + ",1,1")

			indigo.server.log(u"sent \"%s\" %s" % (dev.name, "on"))

		###### TURN OFF ######
		elif action.deviceAction == indigo.kDeviceAction.TurnOff:
			if dev.deviceTypeId == RA_PHANTOM_BUTTON:
				integration_id = dev.pluginProps[PROP_BUTTON]
				sendCmd = ("#DEVICE,1," + str(int(integration_id)-100) + ",4") # Release button
			elif dev.deviceTypeId == RA_PICO:
				pico = dev.pluginProps[PROP_PICO_INTEGRATION_ID]
				button = dev.pluginProps[PROP_PICOBUTTON]
				sendCmd = ("#DEVICE," + pico + "," + button + ",4") # Release button
			elif dev.deviceTypeId == RA_KEYPAD:
				keypad = dev.pluginProps[PROP_KEYPAD]
				keypadButton = dev.pluginProps[PROP_KEYPADBUT]
				if (int(keypadButton) > 80):
					sendCmd = ("#DEVICE," + keypad + "," + str(int(keypadButton)) + ",9,0") # Turn off an LED
				else:
					sendCmd = ("#DEVICE," + keypad + "," + str(int(keypadButton)) + ",4") # Release button
			elif dev.deviceTypeId == RA_DIMMER:
				zone = dev.pluginProps[PROP_ZONE]
				sendCmd = ("#OUTPUT," + zone + ",1,0")
				self.lastBrightness[zone] = 0
			elif dev.deviceTypeId == RA_SHADE:
				shade = dev.pluginProps[PROP_SHADE]
				sendCmd = ("#OUTPUT," + shade + ",1,0")
				self.lastBrightness[shade] = 0
			elif dev.deviceTypeId == RA_SWITCH:
				switch = dev.pluginProps[PROP_SWITCH]
				sendCmd = ("#OUTPUT," + switch + ",1,0")
			elif dev.deviceTypeId == RA_CCI:
				self.debugLog(u"it is a cci")
				cci = dev.pluginProps[PROP_CCI_INTEGRATION_ID]
				component = dev.pluginProps[PROP_COMPONENT]
				sendCmd = ("#DEVICE," + cci +"," + str(int(component)) + ",4")
			elif dev.deviceTypeId == RA_CCO:
				cco = dev.pluginProps[PROP_CCO_INTEGRATION_ID]
				ccoType = dev.pluginProps[PROP_CCO_TYPE]
				if ccoType == "momentary":
					sendCmd = ("#OUTPUT," + cco + ",6")
				else:
					sendCmd = ("#OUTPUT," + cco + ",1,0")
			indigo.server.log(u"sending \"%s\" %s" % (dev.name, "off"))

		###### TOGGLE ######
		elif action.deviceAction == indigo.kDeviceAction.Toggle:
			if dev.deviceTypeId == RA_PHANTOM_BUTTON:
				integration_id = dev.pluginProps[PROP_BUTTON];
				sendCmd = ("#DEVICE,1," + str(int(integration_id)-100) + ",3")
			elif dev.deviceTypeId == RA_KEYPAD:
				keypad = dev.pluginProps[PROP_KEYPAD]
				keypadButton = dev.pluginProps[PROP_KEYPADBUT]
				if (int(keypadButton) > 80):
					if dev.onState == True:
						sendCmd = ("#DEVICE," + keypad + "," + str(int(keypadButton)) + ",9,0") # Turn off an LED
					else:
						sendCmd = ("#DEVICE," + keypad + "," + str(int(keypadButton)) + ",9,1") # Turn on an LED
				else:
					if dev.onState == True:
						sendCmd = ("#DEVICE," + keypad + "," + str(int(keypadButton)) + ",4") # Release button
					else:
						sendCmd = ("#DEVICE," + keypad + "," + str(int(keypadButton)) + ",3") # Press button
			elif dev.deviceTypeId == RA_DIMMER:
				zone = dev.pluginProps[PROP_ZONE]
				if dev.brightness > 0:
					sendCmd = ("#OUTPUT," + zone + ",1,0")
				else:
					sendCmd = ("#OUTPUT," + zone + ",1,100")
			elif dev.deviceTypeId == RA_SHADE:
				shade = dev.pluginProps[PROP_SHADE]
				if dev.brightness > 0:
					sendCmd = ("#OUTPUT," + shade + ",1,0")
				else:
					sendCmd = ("#OUTPUT," + shade + ",1,100")
			elif dev.deviceTypeId == RA_SWITCH:
				switch = dev.pluginProps[PROP_SWITCH]
				if dev.onState == True:
					sendCmd = ("#OUTPUT," + switch + ",1,0")
				else:
					sendCmd = ("#OUTPUT," + switch + ",1,100")
			elif dev.deviceTypeId == RA_CCI:
				self.debugLog(u"it is a cci")
				cci = dev.pluginProps[PROP_CCI_INTEGRATION_ID]
				component = dev.pluginProps[PROP_COMPONENT]
				if dev.onState == True:
					sendCmd = ("#DEVICE," + cci +"," + str(int(component)) + ",4")
				else:
					sendCmd = ("#DEVICE," + cci +"," + str(int(component)) + ",3")
			elif dev.deviceTypeId == RA_CCO:
				cco = dev.pluginProps[PROP_CCO_INTEGRATION_ID]
				ccoType = dev.pluginProps[PROP_CCO_TYPE]
				if ccoType == "momentary":
					sendCmd = ("#OUTPUT," + cco + ",6")
					sendCmd = ("#OUTPUT," + cco + ",1,1")
				else:
					if dev.onState == True:
						sendCmd = ("#OUTPUT," + cco + ",1,0")
					else:
						sendCmd = ("#OUTPUT," + cco + ",1,1")
			indigo.server.log(u"sending \"%s\" %s" % (dev.name, "toggle"))

		###### SET BRIGHTNESS ######
		elif action.deviceAction == indigo.kDeviceAction.SetBrightness:
			if dev.deviceTypeId == RA_DIMMER:
				newBrightness = action.actionValue
				zone = dev.pluginProps[PROP_ZONE]
				sendCmd = ("#OUTPUT," + zone + ",1," + str(newBrightness))
				indigo.server.log(u"sending \"%s\" %s to %d" % (dev.name, "set brightness", newBrightness))
			elif dev.deviceTypeId == RA_SHADE:
				newBrightness = action.actionValue
				shade = dev.pluginProps[PROP_SHADE]
				sendCmd = ("#OUTPUT," + shade + ",1," + str(newBrightness))
				indigo.server.log(u"sending \"%s\" %s to %d" % (dev.name, "set shade open to", newBrightness))

		###### BRIGHTEN BY ######
		elif action.deviceAction == indigo.kDimmerRelayAction.BrightenBy:
			newBrightness = dev.brightness + action.actionValue
			if newBrightness > 100:
				newBrightness = 100
				
			if dev.deviceTypeId == RA_DIMMER:
				zone = dev.pluginProps[PROP_ZONE]
				sendCmd = ("#OUTPUT," + zone + ",1," + str(newBrightness))
				indigo.server.log(u"sending \"%s\" %s to %d" % (dev.name, "set brightness", newBrightness))
			elif dev.deviceTypeId == RA_SHADE:
				shade = dev.pluginProps[PROP_SHADE]
				sendCmd = ("#OUTPUT," + shade + ",1," + str(newBrightness))
				indigo.server.log(u"sending \"%s\" %s to %d" % (dev.name, "set shade open to", newBrightness))

		###### DIM BY ######
		elif action.deviceAction == indigo.kDimmerRelayAction.DimBy:
			newBrightness = dev.brightness - action.actionValue
			if newBrightness < 0:
				newBrightness = 0

			if dev.deviceTypeId == RA_DIMMER:
				zone = dev.pluginProps[PROP_ZONE]
				sendCmd = ("#OUTPUT," + zone + ",1," + str(newBrightness))
				indigo.server.log(u"sending \"%s\" %s to %d" % (dev.name, "set brightness", newBrightness))
			elif dev.deviceTypeId == RA_SHADE:
				shade = dev.pluginProps[PROP_SHADE]
				sendCmd = ("#OUTPUT," + shade + ",1," + str(newBrightness))
				indigo.server.log(u"sending \"%s\" %s to %d" % (dev.name, "set shade open to", newBrightness))

		###### STATUS REQUEST ######
		elif action.deviceAction == indigo.kDeviceAction.RequestStatus:
			if dev.deviceTypeId == RA_PHANTOM_BUTTON:
				integration_id = dev.pluginProps[PROP_BUTTON]
				sendCmd = ("?DEVICE,1," + str(int(integration_id)) + ",9,")
			elif dev.deviceTypeId == RA_KEYPAD:
				keypad = dev.pluginProps[PROP_KEYPAD]
				keypadButton = dev.pluginProps[PROP_KEYPADBUT]
				if (int(keypadButton) > 80):
					sendCmd = ("?DEVICE," + keypad + "," + str(int(keypadButton)) + ",9")
				else:
					sendCmd = ("?DEVICE," + keypad + "," + str(int(keypadButton)+80) + ",9")
			elif dev.deviceTypeId == RA_DIMMER:
				integration_id = dev.pluginProps[PROP_ZONE]
				sendCmd = ("?OUTPUT," + integration_id + ",1,")
			elif dev.deviceTypeId == RA_SHADE:
				integration_id = dev.pluginProps[PROP_SHADE]
				sendCmd = ("?OUTPUT," + integration_id + ",1,")
			elif dev.deviceTypeId == RA_SWITCH:
				integration_id = dev.pluginProps[PROP_SWITCH]
				sendCmd = ("?OUTPUT," + integration_id + ",1,")
			elif dev.deviceTypeId == RA_CCI:
				indigo.server.log(u"This device does not respond to Status Requests")
			elif dev.deviceTypeId == RA_CCO:
				cco = dev.pluginProps[PROP_CCO_INTEGRATION_ID]
				ccoType = dev.pluginProps[PROP_CCO_TYPE]
				if ccoType == "momentary":
					indigo.server.log(u"Momentary CCOs do not respond to Status Requests")
				else:
					sendCmd = ("?OUTPUT," + cco + ",1,")

				
		self._sendCommand(sendCmd)					
		self.debugLog(u"sent \"%s\" %s %s" % (dev.name, dev.onState, sendCmd))

	######################
	# Sensor Action callback
	######################
	def actionControlSensor(self, action, dev):
		indigo.server.log(u"This device does not respond to Status Requests")

	######################
	# Fan Action callback
	######################
	def actionControlSpeedControl(self, action, dev):

		###### SET SPEED ######
		if action.speedControlAction == indigo.kSpeedControlAction.SetSpeedIndex:
			if dev.deviceTypeId == RA_FAN:
				newSpeed = action.actionValue
				fan = dev.pluginProps[PROP_FAN]
				if newSpeed == 0:
					self._sendCommand("#OUTPUT," + fan + ",1,0")
				elif newSpeed == 1:
					self._sendCommand("#OUTPUT," + fan + ",1,25.10")
				elif newSpeed == 2:
					self._sendCommand("#OUTPUT," + fan + ",1,75.30")
				else:
					self._sendCommand("#OUTPUT," + fan + ",1,100")
			indigo.server.log(u"sent \"%s\" %s to %d" % (dev.name, "set fan speed", newSpeed))
		
		###### STATUS REQUEST ######
		elif action.speedControlAction == indigo.kSpeedControlAction.RequestStatus:
				integration_id = dev.pluginProps[PROP_FAN]
				self._sendCommand("?OUTPUT," + integration_id + ",1,")
				indigo.server.log(u"sent \"%s\" %s" % (dev.name, "status request"))

		###### CYCLE SPEED ######
        # Future enhancement
        #elif action.speedControlAction == indigo.kSpeedControlAction.cycleSpeedControlState:

		###### TOGGLE ######
        # Future enhancement
		#elif action.speedControlAction == indigo.kSpeedControlAction.toggle:
		#indigo.server.log(u"sent \"%s\" %s" % (dev.name, "cycle speed"))

	######################
	# HVAC Action callback
	######################

	def actionControlThermostat(self, action, dev):
		integration_id = dev.pluginProps[PROP_THERMO]
		currentCoolSetpoint = dev.coolSetpoint
		currentHeatSetpoint = dev.heatSetpoint
        
		###### SET SETPOINTS ######
		if action.thermostatAction == indigo.kThermostatAction.DecreaseCoolSetpoint:
			newCoolSetpoint = float(currentCoolSetpoint) - 1
			newHeatSetpoint = float(currentHeatSetpoint)
			self._sendCommand("#HVAC," + integration_id + ",2," + str(newHeatSetpoint) + "," + str(newCoolSetpoint))
		elif action.thermostatAction == indigo.kThermostatAction.IncreaseCoolSetpoint:
			newCoolSetpoint = float(currentCoolSetpoint) + 1
			newHeatSetpoint = float(currentHeatSetpoint)
			self._sendCommand("#HVAC," + integration_id + ",2," + str(newHeatSetpoint) + "," + str(newCoolSetpoint))
		elif action.thermostatAction == indigo.kThermostatAction.DecreaseHeatSetpoint:
			newCoolSetpoint = float(currentCoolSetpoint)
			newHeatSetpoint = float(currentHeatSetpoint) - 1
			self._sendCommand("#HVAC," + integration_id + ",2," + str(newHeatSetpoint) + "," + str(newCoolSetpoint))
		elif action.thermostatAction == indigo.kThermostatAction.IncreaseHeatSetpoint:
			newCoolSetpoint = float(currentCoolSetpoint)
			newHeatSetpoint = float(currentHeatSetpoint) + 1
			self._sendCommand("#HVAC," + integration_id + ",2," + str(newHeatSetpoint) + "," + str(newCoolSetpoint))
		elif action.thermostatAction == indigo.kThermostatAction.SetHeatSetpoint:
			newCoolSetpoint = float(currentCoolSetpoint)
			newHeatSetpoint = action.actionValue
			dev.updateStateOnServer("setpointHeat", newHeatSetpoint)
			self._sendCommand("#HVAC," + integration_id + ",2," + str(newHeatSetpoint) + "," + str(newCoolSetpoint) +"\r")
		elif action.thermostatAction == indigo.kThermostatAction.SetCoolSetpoint:
			newCoolSetpoint = action.actionValue
			dev.updateStateOnServer("setpointCool", newCoolSetpoint)
			newHeatSetpoint = float(currentHeatSetpoint)
			self._sendCommand("#HVAC," + integration_id + ",2," + str(newHeatSetpoint) + "," + str(newCoolSetpoint) +"\r")

        ###### SET HVAC MODE ######
		elif action.thermostatAction == indigo.kThermostatAction.SetHvacMode:
			mode = action.actionMode
			if mode == indigo.kHvacMode.Off:
				self._sendCommand("#HVAC," + integration_id + ",3,1")
				indigo.server.log(u"sent \"%s\" %s" % (dev.name, "set hvac mode to Off"))
			elif mode == indigo.kHvacMode.Heat:
				self._sendCommand("#HVAC," + integration_id + ",3,2")
				indigo.server.log(u"sent \"%s\" %s" % (dev.name, "set hvac mode to Heat"))
			elif mode == indigo.kHvacMode.Cool:
				self._sendCommand("#HVAC," + integration_id + ",3,3")
				indigo.server.log(u"sent \"%s\" %s" % (dev.name, "set hvac mode to Cool"))
			elif mode == indigo.kHvacMode.HeatCool:
				self._sendCommand("#HVAC," + integration_id + ",3,4")
				indigo.server.log(u"sent \"%s\" %s" % (dev.name, "set hvac mode to Auto"))

        ###### SET FAN MODE ######
		elif action.thermostatAction == indigo.kThermostatAction.SetFanMode:
			mode = action.actionMode
			if mode == indigo.kFanMode.Auto:
				self._sendCommand("#HVAC," + integration_id + ",4,1")
				indigo.server.log(u"sent \"%s\" %s" % (dev.name, "set fan mode to Auto"))
			elif mode == indigo.kFanMode.AlwaysOn:
				self._sendCommand("#HVAC," + integration_id + ",4,2")
				indigo.server.log(u"sent \"%s\" %s" % (dev.name, "set fan mode to Always On"))

		###### STATUS REQUEST ######
		elif action.thermostatAction == indigo.kThermostatAction.RequestStatusAll:
			self._sendCommand("?HVAC," + integration_id + ",1,") # get temperature
			self._sendCommand("?HVAC," + integration_id + ",2,") # get heat and cool setpoints
			self._sendCommand("?HVAC," + integration_id + ",3,") # get operating mode
			self._sendCommand("?HVAC," + integration_id + ",4,") # get fan mode
			indigo.server.log(u"sent \"%s\" %s" % (dev.name, "status request"))

    #################################
    #
    #  Future versions: implement additional thermostat actions, shades (define as dimmers for now)

