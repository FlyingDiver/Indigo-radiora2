#! /usr/bin/env python
# -*- coding: utf-8 -*-
####################
# Lutron RadioRA 2 server plugin
#
# By Jim Lombardo jim@jimandnoreen.com
# Use as you see fit.  Please share your improvements.
#

import serial
import socket
import telnetlib
import time
import select       # was getting errors on the select.error exception in runConcurrentThread
import logging
import json
import os
import requests
import xml.etree.ElementTree as ET
import threading

# Indigo Custom Device Types
DEV_IP_GATEWAY = "ipGateway"
DEV_SERIAL_GATEWAY = "serialGateway"
DEV_PHANTOM_BUTTON = "ra2PhantomButton"
DEV_DIMMER = "ra2Dimmer"
DEV_SWITCH = "ra2Switch"
DEV_KEYPAD = "ra2Keypad"
DEV_FAN = "ra2Fan"
DEV_THERMO = "ra2Thermo"
DEV_SENSOR = "ra2Sensor"
DEV_CCO = "ra2CCO"
DEV_CCI = "ra2CCI"
DEV_SHADE = "ra2MotorizedShade"
DEV_PICO = "ra2Pico"
DEV_LINKEDDEVICE = "ra2LinkedDevice"
DEV_TIMECLOCKEVENT = "ra2TimeClockEvent"
DEV_GROUP = "ra2Group"

# pluginProps keys
PROP_GATEWAY = "gateway"
PROP_INTEGRATION_ID = "integrationID"
PROP_COMPONENT_ID = "componentID"
PROP_ISBUTTON = "isButton"
PROP_ROOM = "room"
PROP_EVENT = "event"
PROP_GROUP = "group"
PROP_CCO_TYPE = "ccoType"
PROP_LIST_TYPE = "listType"
PROP_KEYPADBUT_DISPLAY_LED_STATE = "keypadButtonDisplayLEDState"
PROP_SUPPORTS_STATUS_REQUEST = "SupportsStatusRequest"
PROP_BUTTONTYPE = "ButtonType"
PROP_OUTPUTTYPE = "OutputType"
PROP_LASTSPEED = "LastSpeed"

# device state keys
ACTUALSPEED = "ActualSpeed"
ONOFF = "onOffState"
SPEEDINDEX = "speedIndex"

# deprecated properties, all now use PROP_INTEGRATION_ID
PROP_REPEATER = "repeater"
PROP_ZONE = "zone"
PROP_SWITCH = "switch"
PROP_KEYPAD = "keypad"
PROP_FAN = "fan"
PROP_THERMO = "thermo"
PROP_SENSOR = "sensor"
PROP_SHADE = "shade"
PROP_CCO_INTEGRATION_ID = "ccoIntegrationID"
PROP_CCI_INTEGRATION_ID = "cciIntegrationID"
PROP_PICO_INTEGRATION_ID = "picoIntegrationID"

# deprecated properties, all now use PROP_COMPONENT_ID
PROP_KEYPADBUT = "keypadButton"
PROP_PICOBUTTON = "picoButton"
PROP_COMPONENT = "component"
PROP_BUTTON = "button"

# delay amount for detecting double/triple clicks
CLICK_DELAY = 1.0

########################################
class IPGateway:
########################################

    def __init__(self, plugin, dev):
        self.logger = logging.getLogger("Plugin.IPGateway")
        self.dev = dev
        self.connected = False
        dev.updateStateOnServer(key="status", value="Disconnected")
        dev.updateStateImageOnServer(indigo.kStateImageSel.SensorOff)

    def startup(self):
        self.logger.info(u"{}: Running IP Startup".format(self.dev.name))

        host = self.dev.pluginProps["address"]
        port = int(self.dev.pluginProps["port"])

        self.timeout = 35   # Under some conditions Smart Bridge Pro takes a long time to connect
        
        try:
            self.logger.info(u"Connecting via IP to {}:{}".format(host, port))
            self.connIP = telnetlib.Telnet(host, port, self.timeout)
        except socket.timeout:
            self.logger.error(u"{}: Unable to connect to Lutron gateway. Timed out.".format(self.dev.name))
            self.dev.updateStateImageOnServer(indigo.kStateImageSel.SensorTripped)
            return
                    
        txt = self.connIP.read_until(" ", self.timeout)
        self.logger.debug(u"{}: self.connIP.read: {}".format(self.dev.name, txt))

        if 'login' not in txt:
            self.logger.debug(u"{}: No login prompt, unable to send username".format(self.dev.name))
            self.dev.updateStateImageOnServer(indigo.kStateImageSel.SensorTripped)
            return

        self.logger.debug(u"{}: Sending username".format(self.dev.name))
        self.connIP.write(str(self.dev.pluginProps["username"]) + "\r\n")

        txt = self.connIP.read_until(" ", self.timeout)
        self.logger.debug(u"{}: self.connIP.read: {}".format(self.dev.name, txt))
        if 'password' not in txt:
            self.logger.debug(u"{}: No password prompt, unable to send password".format(self.dev.name))
            self.dev.updateStateImageOnServer(indigo.kStateImageSel.SensorTripped)
            return

        self.logger.debug(u"{}: Sending password".format(self.dev.name))
        self.connIP.write(str(self.dev.pluginProps["password"]) + "\r\n")
                
        self.logger.debug(u"{}: Login process complete, connected".format(self.dev.name))
        self.timeout = 5   # Reset the timeout to something reasonable
        self.connected = True
        self.dev.updateStateOnServer(key="status", value="Connected")
        self.dev.updateStateImageOnServer(indigo.kStateImageSel.SensorOn)
        
    def poll(self):

        try:
            if not self.connected:
                self.startup()

            try:
                return self.connIP.read_until("\n", self.timeout)
            except:
                pass
            
        except EOFError, e:
            self.logger.error(u"EOFError: %s" % e.message)
            if ('telnet connection closed' in e.message):
                self.connected = False
                self.dev.updateStateOnServer(key="status", value="Disconnected")
                self.dev.updateStateImageOnServer(indigo.kStateImageSel.SensorTripped)
                self.sleep(10)
        except AttributeError, e:
            self.logger.debug(u"AttributeError: %s" % e.message)
        except select.error, e:
            self.logger.debug(u"Disconnected while listening: %s" % e.message)

    def send(self, cmd):
        self.logger.debug(u"Sending network command:  %s" % cmd)
        cmd = cmd + "\r\n"
        try:
            self.connIP.write(str(cmd))
        except Exception, e:
            self.logger.warning(u"Error sending IP command, resetting connection:  %s", e)
            self.connIP.close()
            self.connected = False
            self.dev.updateStateOnServer(key="status", value="Connected")
            self.dev.updateStateImageOnServer(indigo.kStateImageSel.SensorOn)

    def fetchXML(self):

        host = self.dev.pluginProps["address"]

        s = requests.Session()
        r = s.get('http://' + host + '/login?login=lutron&password=lutron')
        r = s.get('http://' + host + '/DbXmlInfo.xml')
        
        return r.text

########################################
class SerialGateway:
########################################

    def __init__(self, plugin, dev):
        self.logger = logging.getLogger("Plugin.SerialGateway")
        self.dev = dev
        self.plugin = plugin
        self.connected = False
        dev.updateStateOnServer(key="status", value="Disconnected")
        dev.updateStateImageOnServer(indigo.kStateImageSel.SensorOff)

        self.connSerial = {}
        self.command = ''
        

    def startup(self):
        self.logger.info(u"{}: Running Serial Startup".format(self.dev.name))

        self.serialUrl = self.dev.pluginProps.get(u"serialPort", None)
        self.logger.info(u"{}: Serial Port URL is: {}".format(self.dev.name, self.serialUrl))

        try:
            self.connSerial = self.plugin.openSerial(u"Lutron Gateway", self.serialUrl, 9600, stopbits=1, timeout=2, writeTimeout=1)
            if self.connSerial is None:
                self.logger.error(u"{}: Failed to open serial port".format(self.dev.name))
                self.dev.updateStateOnServer(key="status", value="Failed")
                self.dev.updateStateImageOnServer(indigo.kStateImageSel.SensorTripped)
                return

        except Exception, e:
            self.logger.debug(u"Error opening Serial port: {}".format(e.message))

        self.connected = True
        self.dev.updateStateOnServer(key="status", value="Connected")
        self.dev.updateStateImageOnServer(indigo.kStateImageSel.SensorOn)

        # Disable main repeater terminal prompt
        self.send("#MONITORING,12,2")

        # Enable main repeater HVAC monitoring
        self.send("#MONITORING,17,1")

        # Enable main repeater monitoring param 18 (undocumented but seems to be enabled by default for ethernet connections)
        self.send("#MONITORING,18,1")


    def poll(self):

        if not self.connected:
            self.startup()

        s = self.connSerial.read()
        if len(s) > 0:
            
            if s == '\r':               # RadioRA 2 messages are always terminated with CRLF
                tmp = self.command
                self.command = ''
                return tmp
            else:
                self.command += s

    def send(self, cmd):
        self.logger.debug(u"Sending serial command: %s" % cmd)
        cmd = cmd + "\r"
        self.connSerial.write(str(cmd))



########################################
class Plugin(indigo.PluginBase):
########################################
    def __init__(self, pluginId, pluginDisplayName, pluginVersion, pluginPrefs):
        indigo.PluginBase.__init__(self, pluginId, pluginDisplayName, pluginVersion, pluginPrefs)

        pfmt = logging.Formatter('%(asctime)s.%(msecs)03d\t[%(levelname)8s] %(name)20s.%(funcName)-25s%(msg)s', datefmt='%Y-%m-%d %H:%M:%S')
        self.plugin_file_handler.setFormatter(pfmt)

        try:
            self.logLevel = int(self.pluginPrefs[u"logLevel"])
        except:
            self.logLevel = logging.INFO
        self.indigo_log_handler.setLevel(self.logLevel)
        self.logger.debug(u"logLevel = " + str(self.logLevel))

        self.gateways = {}
        
        self.phantomButtons = {}
        self.keypads = {}
        self.dimmers = {}
        self.switches = {}
        self.lastBrightness = {}
        self.fans = {}
        self.thermos = {}
        self.sensors = {}
        self.ccis = {}
        self.ccos = {}
        self.shades = {}
        self.picos = {}
        self.events = {}
        self.groups = {}
                
        self.eventTriggers = { }
        self.groupTriggers = { }
        self.buttonTriggers = { }
        
        self.roomButtonTree = {}
        self.lastKeyTime = time.time()
        self.lastKeyAddress = ""
        self.key_taps = 0
        
        self.threadLock = threading.Lock()  # for background data fetch

    def startup(self):
        self.logger.info(u"Starting up Lutron")

        savedList = indigo.activePlugin.pluginPrefs.get(u"linkedDevices", None)
        if savedList:
            self.linkedDeviceList = json.loads(savedList)
        else:
            self.linkedDeviceList = {}        
        self.logLinkedDevices()

        indigo.devices.subscribeToChanges()
        
        IP = indigo.activePlugin.pluginPrefs.get(u"IP", "None")
        if IP != "None":

            if IP:
                address = self.pluginPrefs["ip_address"]
                name = "Lutron IP Gateway"
                props = {
                    "port" : self.pluginPrefs["ip_port"], 
                    "username" : self.pluginPrefs["ip_username"], 
                    "password" : self.pluginPrefs["ip_password"] 
                }
                self.logger.info("Creating new IP Gateway device @ {}:{}".format(address, self.pluginPrefs["ip_port"]))
                try:
                    newDevice = indigo.device.create(indigo.kProtocol.Plugin, address=address, name=name, deviceTypeId=DEV_IP_GATEWAY, props=props)
                except Exception, e:
                    self.logger.error("Error calling indigo.device.create(): %s" % (e.message))
                else:
                    del indigo.activePlugin.pluginPrefs[u"IP"]
                    del indigo.activePlugin.pluginPrefs[u"ip_address"]
                    del indigo.activePlugin.pluginPrefs[u"ip_port"]
                    del indigo.activePlugin.pluginPrefs[u"ip_username"]
                    del indigo.activePlugin.pluginPrefs[u"ip_password"]
                                
            else:
            
                address = self.pluginPrefs["serialport"]
                name = "Lutron Serial Gateway"
                props = {
                    "serialport" : address
                }
                self.logger.info("Creating new Serial Gateway device @ {}".format(address))
                try:
                    newDevice = indigo.device.create(indigo.kProtocol.Plugin, address=address, name=name, deviceTypeId=DEV_SERIAL_GATEWAY, props=props)
                except Exception, e:
                    self.logger.error("Error calling indigo.device.create(): %s" % (e.message))
                else:
                    del indigo.activePlugin.pluginPrefs[u"IP"]
                    del indigo.activePlugin.pluginPrefs[u"serialport"]
            
            self.newGateway = newDevice.id
            
    def shutdown(self):
        self.logger.info(u"Shutting down Lutron")

  
    ################################################################################
    #
    # delegate methods for indigo.devices.subscribeToChanges()
    #
    ################################################################################

    def deviceDeleted(self, delDevice):
        indigo.PluginBase.deviceDeleted(self, delDevice)

        for linkID in self.linkedDeviceList.keys():
            linkItem = self.linkedDeviceList[linkID]
            if (delDevice.id == int(linkItem["buttonDevice"])) or (delDevice.id == int(linkItem["buttonLEDDevice"])) or (delDevice.id == int(linkItem["controlledDevice"])):
                self.logger.info(u"A linked device ({}) has been deleted.  Deleting link: {}".format(delDevice.name, linkItem["name"]))
                del self.linkedDeviceList[linkID]
                self.logLinkedDevices()

                indigo.activePlugin.pluginPrefs[u"linkedDevices"] = json.dumps(self.linkedDeviceList)
                

    def deviceUpdated(self, oldDevice, newDevice):
        indigo.PluginBase.deviceUpdated(self, oldDevice, newDevice)

        for linkName, linkItem in self.linkedDeviceList.iteritems():
            controlledDevice = indigo.devices[int(linkItem["controlledDevice"])]
            buttonDevice = indigo.devices[int(linkItem["buttonDevice"])]
            
            if oldDevice.id == controlledDevice.id:

                self.logger.debug(u"A linked device ({}) has been updated: {}".format(controlledDevice.name, controlledDevice.onState))
                try:
                    buttonLEDDevice = indigo.devices[int(linkItem["buttonLEDDevice"])]
                except:
                    pass
                else:
                    if controlledDevice.onState:
                        indigo.device.turnOn(buttonLEDDevice.id)
                    else:
                        indigo.device.turnOff(buttonLEDDevice.id)
                    
          
    ####################

    def triggerStartProcessing(self, trigger):
                                                
        if  trigger.pluginTypeId == "keypadButtonPress":
            try:
                clicks = trigger.pluginProps["clicks"]
            except:
                clicks = trigger.pluginProps["clicks"] = "1"
        
            try:
                deviceID = trigger.pluginProps["deviceID"]
                componentID = trigger.pluginProps["componentID"]
            except:
                try:
                    buttonID = trigger.pluginProps["buttonID"]
                    buttonAddress = indigo.devices[int(buttonID)].address
                    parts = buttonAddress.split(".")
                    deviceID = trigger.pluginProps["deviceID"] =  parts[0]
                    componentID = trigger.pluginProps["componentID"] = parts[1]
                except:
                    self.logger.error("keypadButtonPress Trigger  %s (%s) missing deviceID/componentID/buttonID: %s" % (trigger.name, trigger.id, str(trigger.pluginProps)))
                    return

            self.logger.debug("Adding Button Trigger '{}', deviceID = {}, componentID = {}, clicks =  {}".format(trigger.name, deviceID, componentID, clicks))
            self.buttonTriggers[trigger.id] = trigger


        elif trigger.pluginTypeId == "timeClockEvent":
            try:
                event = trigger.pluginProps[PROP_EVENT]
            except:
                self.logger.error(u"Timeclock Event Trigger %s (%s) does not contain event: %s" % (trigger.name, trigger.id, str(trigger.pluginProps)))
                return

            self.logger.debug("Adding Event Trigger {}, event = {}".format(trigger.name, event))
            self.eventTriggers[trigger.id] = trigger
        
        elif trigger.pluginTypeId == "groupEvent":
            try:
                group = trigger.pluginProps[PROP_GROUP]
            except:
                self.logger.error(u"Group Trigger %s (%s) does not contain group: %s" % (trigger.name, trigger.id, str(trigger.pluginProps)))
                return
        
            self.logger.debug("Adding Group Trigger {}, group = {}".format(trigger.name, group))
            self.groupTriggers[trigger.id] = trigger
        
        else:
            self.logger.error(u"triggerStartProcessing: Trigger '{}' is unknown type: {}" % (trigger.name, trigger.pluginTypeId))
                      
                      
    def triggerStopProcessing(self, trigger):

        self.logger.debug(u"Removing Trigger %s (%d)" % (trigger.name, trigger.id))
        if  trigger.pluginTypeId == "keypadButtonPress":
            del self.buttonTriggers[trigger.id]
        elif  trigger.pluginTypeId == "timeClockEvent":
            del self.eventTriggers[trigger.id]
        elif  trigger.pluginTypeId == "groupEvent":
            del self.groupTriggers[trigger.id]
        else:
            self.logger.error(u"triggerStopProcessing: Trigger %s (%s) is unknown type: %s" % (trigger.name, trigger.id, trigger.pluginTypeId))
                      


    def eventTriggerCheck(self, info):

        self.logger.debug(u"eventTriggerCheck: event %s" % (info))

        for triggerId, trigger in self.eventTriggers.iteritems():

            event = trigger.pluginProps[PROP_EVENT]
            if event != info:
                self.logger.threaddebug(u"eventTriggerCheck: Skipping Trigger %s (%s), wrong event: %s" % (trigger.name, trigger.id, event))
                continue

            self.logger.debug(u"eventTriggerCheck: Executing Trigger %s (%s), event: %s" % (trigger.name, trigger.id, info))
            indigo.trigger.execute(trigger)

    def groupTriggerCheck(self, groupID, status):

        self.logger.debug(u"groupTriggerCheck: group %s %s" % (groupID, status))

        for triggerId, trigger in self.groupTriggers.iteritems():

            group = trigger.pluginProps[PROP_GROUP]
            occupancy = trigger.pluginProps["occupancyPopUp"]
            if (group != groupID) or (occupancy != status):
                self.logger.threaddebug(u"groupTriggerCheck: Skipping Trigger %s (%s), wrong group or status: %s, %s" % (trigger.name, trigger.id, group, occupancy))
                continue

            self.logger.info(u"groupTriggerCheck: Executing Trigger %s (%s), group %s, status %s" % (trigger.name, trigger.id, groupID, status))
            indigo.trigger.execute(trigger)

    def buttonTriggerCheck(self, devID, compID):

        triggerAddress = "{}.{}".format(devID, compID)
        self.logger.debug(u"buttonTriggerCheck: devID: {}, compID: {}, address: {}".format(devID, compID, triggerAddress))
        
        # check for linked devices
        
        for linkID, linkItem in self.linkedDeviceList.iteritems():
            controlledDevice = indigo.devices[int(linkItem["controlledDevice"])]
            buttonAddress = linkItem["buttonAddress"]
            if buttonAddress == triggerAddress:
                self.logger.debug(u"Linked Device Match, buttonAddress: {}, controlledDevice: {}".format(buttonAddress, controlledDevice.id))
                indigo.device.toggle(controlledDevice.id)


        # and check for multiple taps
        
        if (triggerAddress == self.lastKeyAddress) and (time.time() < (self.lastKeyTime + CLICK_DELAY)):
            self.key_taps += 1
        else:
            self.key_taps = 1
            
        self.lastKeyAddress = triggerAddress
        self.lastKeyTime = time.time()
            
        # Look for triggers that match this button

        for triggerId, trigger in self.buttonTriggers.iteritems():
            try:
                clicks = trigger.pluginProps["clicks"]
            except:
                clicks = "1"
        
            try:
                deviceID = trigger.pluginProps["deviceID"]
                componentID = trigger.pluginProps["componentID"]
            except:
                try:
                    buttonID = trigger.pluginProps["buttonID"]
                    buttonAddress = indigo.devices[int(buttonID)].address
                    parts = buttonAddress.split(".")
                    deviceID =  parts[0]
                    componentID = parts[1]
                except:
                    self.logger.error(u"keypadButtonPress Trigger '{}' missing deviceID/componentID/buttonID: {}".format(trigger.name, trigger.pluginProps))
                    return

            if (deviceID != devID) or (componentID != compID):
                self.logger.threaddebug(u"buttonTriggerCheck: Skipping Trigger '{}', wrong keypad button: {}, {}".format(trigger.name, deviceID, componentID))
                continue
                
            if self.key_taps != int(clicks):
                self.logger.threaddebug(u"buttonTriggerCheck: Skipping Trigger {}, wrong click count: {}".format(trigger.name, clicks))
                continue

            self.logger.debug(u"buttonTriggerCheck: Executing Trigger '{}', keypad button: {}.{}".format(trigger.name, deviceID, componentID))
            indigo.trigger.execute(trigger)
        
            
    ####################

    def update_device_property(self, dev, propertyname, new_value = ""):
        newProps = dev.pluginProps
        newProps.update( {propertyname : new_value} )
        dev.replacePluginPropsOnServer(newProps)
        return None

    def remove_device_property(self, dev, propertyname):
        newProps = dev.pluginProps
        del newProps[propertyname]
        dev.replacePluginPropsOnServer(newProps)
        return None

    # prevent deviceStartComm/deviceStopComm on property changes
    def didDeviceCommPropertyChange(self, origDev, newDev):
        if origDev.deviceTypeId != newDev.deviceTypeId:
            return True
        else:
            return False      

              
    def deviceStartComm(self, dev):
    
        if dev.deviceTypeId == DEV_IP_GATEWAY:
            gateway = IPGateway(self, dev)
            self.gateways[dev.id] = gateway            
            dev.updateStateOnServer(key="status", value="None")
            dev.updateStateImageOnServer(indigo.kStateImageSel.SensorOff)
            gateway.startup()
            
        elif dev.deviceTypeId == DEV_SERIAL_GATEWAY:
            gateway = SerialGateway(self, dev)
            self.gateways[dev.id] = gateway            
            dev.updateStateOnServer(key="status", value="None")
            dev.updateStateImageOnServer(indigo.kStateImageSel.SensorOff)
            gateway.startup()
                      
        elif dev.deviceTypeId == DEV_PHANTOM_BUTTON:
            if dev.pluginProps.get(PROP_REPEATER, None):
                self.update_device_property(dev, PROP_INTEGRATION_ID, dev.pluginProps[PROP_REPEATER])
                self.remove_device_property(dev, PROP_REPEATER)
                self.logger.info(u"{}: Updated repeater property to IntegrationID".format(dev.name))
            elif dev.pluginProps.get(PROP_INTEGRATION_ID, None) == None:
                self.update_device_property(dev, PROP_INTEGRATION_ID, "1")
                self.logger.info(u"{}: Added IntegrationID property".format(dev.name))
                
            if dev.pluginProps.get(PROP_BUTTON, None):
                self.update_device_property(dev, PROP_COMPONENT_ID, dev.pluginProps[PROP_BUTTON])
                self.remove_device_property(dev, PROP_BUTTON)
                self.logger.info(u"{}: Updated button property to componentID".format(dev.name))
                
            if dev.pluginProps.get(PROP_ISBUTTON, None) == None:
                self.update_device_property(dev, PROP_ISBUTTON, "True")
                self.logger.info(u"{}: Added isButton property".format(dev.name))

            if dev.pluginProps.get(PROP_GATEWAY, None) == None:
                self.update_device_property(dev, PROP_GATEWAY, self.newGateway)
                self.logger.info(u"{}: Added Gateway property".format(dev.name))

            address = u"{}:{}.{}".format(dev.pluginProps[PROP_GATEWAY], dev.pluginProps[PROP_INTEGRATION_ID], dev.pluginProps[PROP_COMPONENT_ID])
            self.phantomButtons[address] = dev
            self.update_device_property(dev, "address", address)
            
        elif dev.deviceTypeId == DEV_DIMMER:
            if dev.pluginProps.get(PROP_INTEGRATION_ID, None) == None:
                self.update_device_property(dev, PROP_INTEGRATION_ID, dev.pluginProps[PROP_ZONE])
                self.remove_device_property(dev, PROP_ZONE)
                self.logger.info(u"{}: Updated zone property to IntegrationID".format(dev.name))

            if dev.pluginProps.get(PROP_GATEWAY, None) == None:
                self.update_device_property(dev, PROP_GATEWAY, self.newGateway)
                self.logger.info(u"{}: Added Gateway property".format(dev.name))

            address = u"{}:{}".format(dev.pluginProps[PROP_GATEWAY], dev.pluginProps[PROP_INTEGRATION_ID])
            self.dimmers[address] = dev
            self.update_device_property(dev, "address", address)
            
        elif dev.deviceTypeId == DEV_SHADE:
            if dev.pluginProps.get(PROP_INTEGRATION_ID, None) == None:
                self.update_device_property(dev, PROP_INTEGRATION_ID, dev.pluginProps[PROP_SHADE])
                self.remove_device_property(dev, PROP_SHADE)
                self.logger.info(u"{}: Updated zone property to IntegrationID".format(dev.name))

            if dev.pluginProps.get(PROP_GATEWAY, None) == None:
                self.update_device_property(dev, PROP_GATEWAY, self.newGateway)
                self.logger.info(u"{}: Added Gateway property".format(dev.name))

            address = u"{}:{}".format(dev.pluginProps[PROP_GATEWAY], dev.pluginProps[PROP_INTEGRATION_ID])
            self.shades[address] = dev
            self.update_device_property(dev, "address", address)
            dev.updateStateImageOnServer( indigo.kStateImageSel.None)
            
        elif dev.deviceTypeId == DEV_SWITCH:
            if dev.pluginProps.get(PROP_INTEGRATION_ID, None) == None:
                self.update_device_property(dev, PROP_INTEGRATION_ID, dev.pluginProps[PROP_SWITCH])
                self.remove_device_property(dev, PROP_SWITCH)
                self.logger.info(u"{}: Updated switch property to IntegrationID".format(dev.name))

            if dev.pluginProps.get(PROP_GATEWAY, None) == None:
                self.update_device_property(dev, PROP_GATEWAY, self.newGateway)
                self.logger.info(u"{}: Added Gateway property".format(dev.name))

            address = u"{}:{}".format(dev.pluginProps[PROP_GATEWAY], dev.pluginProps[PROP_INTEGRATION_ID])
            self.switches[address] = dev
            self.update_device_property(dev, "address", address)
            
        elif dev.deviceTypeId == DEV_FAN:
            if dev.pluginProps.get(PROP_INTEGRATION_ID, None) == None:
                self.update_device_property(dev, PROP_INTEGRATION_ID, dev.pluginProps[PROP_FAN])
                self.remove_device_property(dev, PROP_FAN)
                self.logger.info(u"{}: Updated fan property to IntegrationID".format(dev.name))

            if dev.pluginProps.get(PROP_GATEWAY, None) == None:
                self.update_device_property(dev, PROP_GATEWAY, self.newGateway)
                self.logger.info(u"{}: Added Gateway property".format(dev.name))

            address = u"{}:{}".format(dev.pluginProps[PROP_GATEWAY], dev.pluginProps[PROP_INTEGRATION_ID])
            self.fans[address] = dev
            self.update_device_property(dev, "address", address)

        elif dev.deviceTypeId == DEV_THERMO:
            if dev.pluginProps.get(PROP_INTEGRATION_ID, None) == None:
                self.update_device_property(dev, PROP_INTEGRATION_ID, dev.pluginProps[PROP_THERMO])
                self.remove_device_property(dev, PROP_THERMO)
                self.logger.info(u"{}: Updated thermo property to IntegrationID".format(dev.name))

            if dev.pluginProps.get(PROP_GATEWAY, None) == None:
                self.update_device_property(dev, PROP_GATEWAY, self.newGateway)
                self.logger.info(u"{}: Added Gateway property".format(dev.name))

            address = u"{}:{}".format(dev.pluginProps[PROP_GATEWAY], dev.pluginProps[PROP_INTEGRATION_ID])
            self.thermos[address] = dev
            self.update_device_property(dev, "address", address)
            
        elif dev.deviceTypeId == DEV_KEYPAD:
            if dev.pluginProps.get(PROP_INTEGRATION_ID, None) == None:
                self.update_device_property(dev, PROP_INTEGRATION_ID, dev.pluginProps[PROP_KEYPAD])
                self.remove_device_property(dev, PROP_KEYPAD)
                self.logger.info(u"{}: Updated keypad property to IntegrationID".format(dev.name))

            if dev.pluginProps.get(PROP_KEYPADBUT, None):
                self.update_device_property(dev, PROP_COMPONENT_ID, dev.pluginProps[PROP_KEYPADBUT])
                self.remove_device_property(dev, PROP_KEYPADBUT)
                self.logger.info(u"{}: Updated keypadButton property to componentID".format(dev.name))
                
            if (dev.pluginProps.get(PROP_ISBUTTON, None) == None) and (int(dev.pluginProps[PROP_COMPONENT_ID]) < 80):
                self.update_device_property(dev, PROP_ISBUTTON, new_value = "True")
                self.logger.info(u"%s: Added isButton property" % (dev.name))
                
            if dev.pluginProps.get(PROP_GATEWAY, None) == None:
                self.update_device_property(dev, PROP_GATEWAY, self.newGateway)
                self.logger.info(u"{}: Added Gateway property".format(dev.name))

            address = u"{}:{}.{}".format(dev.pluginProps[PROP_GATEWAY], dev.pluginProps[PROP_INTEGRATION_ID], dev.pluginProps[PROP_COMPONENT_ID])
            self.update_device_property(dev, "address", address)
            if int(dev.pluginProps[PROP_COMPONENT_ID]) > 80:
                self.update_device_property(dev, PROP_KEYPADBUT_DISPLAY_LED_STATE, new_value = dev.pluginProps[PROP_KEYPADBUT_DISPLAY_LED_STATE])
            else:
                self.update_device_property(dev, PROP_KEYPADBUT_DISPLAY_LED_STATE, new_value = False)
            self.keypads[address] = dev

        elif dev.deviceTypeId == DEV_SENSOR:
            if dev.pluginProps.get(PROP_INTEGRATION_ID, None) == None:
                self.update_device_property(dev, PROP_INTEGRATION_ID, dev.pluginProps[PROP_SENSOR])
                self.remove_device_property(dev, PROP_SENSOR)
                self.logger.info(u"{}: Updated sensor property to IntegrationID".format(dev.name))

            if dev.pluginProps.get(PROP_GATEWAY, None) == None:
                self.update_device_property(dev, PROP_GATEWAY, self.newGateway)
                self.logger.info(u"{}: Added Gateway property".format(dev.name))

            address = u"{}:{}".format(dev.pluginProps[PROP_GATEWAY], dev.pluginProps[PROP_INTEGRATION_ID])
            self.sensors[address] = dev
            self.update_device_property(dev, "address", address)

        elif dev.deviceTypeId == DEV_CCI:
            if dev.pluginProps.get(PROP_INTEGRATION_ID, None) == None:
                self.update_device_property(dev, PROP_INTEGRATION_ID, dev.pluginProps[PROP_CCI_INTEGRATION_ID])
                self.remove_device_property(dev, PROP_CCI_INTEGRATION_ID)
                self.logger.info(u"{}: Updated cciIntegrationID property to IntegrationID".format(dev.name))

            if dev.pluginProps.get(PROP_COMPONENT, None):
                self.update_device_property(dev, PROP_COMPONENT_ID, dev.pluginProps[PROP_COMPONENT])
                self.remove_device_property(dev, PROP_COMPONENT)
                self.logger.info(u"{}: Updated cciCompoment property to componentID".format(dev.name))
                
            if dev.pluginProps.get(PROP_GATEWAY, None) == None:
                self.update_device_property(dev, PROP_GATEWAY, self.newGateway)
                self.logger.info(u"{}: Added Gateway property".format(dev.name))

            address = u"{}:{}.{}".format(dev.pluginProps[PROP_GATEWAY], dev.pluginProps[PROP_INTEGRATION_ID], dev.pluginProps[PROP_COMPONENT_ID])
            self.ccis[address] = dev
            self.update_device_property(dev, "address", address)

        elif dev.deviceTypeId == DEV_CCO:
            if dev.pluginProps.get(PROP_INTEGRATION_ID, None) == None:
                self.update_device_property(dev, PROP_INTEGRATION_ID, dev.pluginProps[PROP_CCO_INTEGRATION_ID])
                self.remove_device_property(dev, PROP_CCO_INTEGRATION_ID)
                self.logger.info(u"{}: Updated ccoIntegrationID property to IntegrationID".format(dev.name))

            if dev.pluginProps.get(PROP_GATEWAY, None) == None:
                self.update_device_property(dev, PROP_GATEWAY, self.newGateway)
                self.logger.info(u"{}: Added Gateway property".format(dev.name))

            address = u"{}:{}".format(dev.pluginProps[PROP_GATEWAY], dev.pluginProps[PROP_INTEGRATION_ID])
            self.ccos[address] = dev
            self.update_device_property(dev, "address", address)

            ccoType = dev.pluginProps[PROP_CCO_TYPE]
            if ccoType == "momentary":
                dev.updateStateOnServer(ONOFF, False)
            else:
                self.update_device_property(dev, PROP_SUPPORTS_STATUS_REQUEST, new_value = True)
            
        elif dev.deviceTypeId == DEV_PICO:
            if dev.pluginProps.get(PROP_INTEGRATION_ID, None) == None:
                self.update_device_property(dev, PROP_INTEGRATION_ID, dev.pluginProps[PROP_PICO_INTEGRATION_ID])
                self.remove_device_property(dev, PROP_PICO_INTEGRATION_ID)
                self.logger.info(u"{}: Updated picoIntegrationID property to IntegrationID".format(dev.name))

            if dev.pluginProps.get(PROP_ISBUTTON, None) == None:
                self.update_device_property(dev, PROP_ISBUTTON, new_value = "True")
                self.logger.info(u"%s: Added isButton property" % (dev.name))
                
            if dev.pluginProps.get(PROP_PICOBUTTON, None):
                self.update_device_property(dev, PROP_COMPONENT_ID, dev.pluginProps[PROP_PICOBUTTON])
                self.remove_device_property(dev, PROP_PICOBUTTON)
                self.logger.info(u"{}: Updated keypadButton property to componentID".format(dev.name))
                
            if dev.pluginProps.get(PROP_GATEWAY, None) == None:
                self.update_device_property(dev, PROP_GATEWAY, self.newGateway)
                self.logger.info(u"{}: Added Gateway property".format(dev.name))

            address = u"{}:{}.{}".format(dev.pluginProps[PROP_GATEWAY], dev.pluginProps[PROP_INTEGRATION_ID], dev.pluginProps[PROP_COMPONENT_ID])
            self.picos[address] = dev
            self.update_device_property(dev, "address", address)

        elif dev.deviceTypeId == DEV_TIMECLOCKEVENT:

            if dev.pluginProps.get(PROP_GATEWAY, None) == None:
                self.update_device_property(dev, PROP_GATEWAY, self.newGateway)
                self.logger.info(u"{}: Added Gateway property".format(dev.name))

            address = u"{}:Event.{}".format(dev.pluginProps[PROP_GATEWAY], dev.pluginProps[PROP_EVENT])
            self.events[address] = dev
            self.update_device_property(dev, "address", address)

        elif dev.deviceTypeId == DEV_GROUP:

            if dev.pluginProps.get(PROP_GATEWAY, None) == None:
                self.update_device_property(dev, PROP_GATEWAY, self.newGateway)
                self.logger.info(u"{}: Added Gateway property".format(dev.name))

            address = u"{}:Group.{}".format(dev.pluginProps[PROP_GATEWAY], dev.pluginProps[PROP_GROUP])
            self.groups[address] = dev
            self.update_device_property(dev, "address", address)

        elif dev.deviceTypeId == DEV_LINKEDDEVICE:
        
            # migrate devices then delete them
            
            buttonDeviceId = int(dev.pluginProps["buttonDevice"])
            buttonLEDDeviceId = int(dev.pluginProps["buttonLEDDevice"])
            controlledDeviceId = int(dev.pluginProps["controlledDevice"])
            buttonAddress = dev.pluginProps["buttonAddress"]

            linkID = "{}-{}".format(buttonDeviceId, controlledDeviceId)
            linkItem = {"name" : linkID, "buttonDevice" : buttonDeviceId, "buttonLEDDevice" : buttonLEDDeviceId, "controlledDevice" : controlledDeviceId, "buttonAddress" : buttonAddress}
            self.logger.debug(u"Adding linkItem {}: {}".format(linkID, linkItem))
            self.linkedDeviceList[linkID] = linkItem
            self.logLinkedDevices()
            indigo.activePlugin.pluginPrefs[u"linkedDevices"] = json.dumps(self.linkedDeviceList)
            indigo.device.delete(dev.id)

        else:
            self.logger.error(u"{}: deviceStartComm: Unknown device type: {}".format(dev.name, dev.deviceTypeId))
            return
        
        if (dev.pluginProps.get(PROP_ISBUTTON, None)):      # it's a button, so put it in the tree
            try:
                roomName = dev.pluginProps[PROP_ROOM]
            except:
                roomName = u"Unknown"
            try:
                room = self.roomButtonTree[roomName]
            except:
                room = {}
                self.roomButtonTree[roomName] = room
            self.roomButtonTree[roomName][dev.id] = dev.name
        
        
    def deviceStopComm(self, dev):
        if dev.deviceTypeId == DEV_IP_GATEWAY:
            del self.gateways[dev.id]
            
        elif dev.deviceTypeId == DEV_SERIAL_GATEWAY:
            del self.gateways[dev.id]
            
        elif dev.deviceTypeId == DEV_PHANTOM_BUTTON:
            del self.phantomButtons[dev.address]

        elif dev.deviceTypeId == DEV_DIMMER:
            del self.dimmers[dev.address]

        elif dev.deviceTypeId == DEV_SHADE:
            del self.shades[dev.address]

        elif dev.deviceTypeId == DEV_SWITCH:
            del self.switches[dev.address]

        elif dev.deviceTypeId == DEV_KEYPAD:
            del self.keypads[dev.address]

        elif dev.deviceTypeId == DEV_FAN:
            del self.fans[dev.address]

        elif dev.deviceTypeId == DEV_THERMO:
            del self.thermos[dev.address]

        elif dev.deviceTypeId == DEV_SENSOR:
            del self.sensors[dev.address]

        elif dev.deviceTypeId == DEV_CCI:
            del self.ccis[dev.address]

        elif dev.deviceTypeId == DEV_CCO:
            del self.ccos[dev.address]

        elif dev.deviceTypeId == DEV_PICO:
            del self.picos[dev.address]

        elif dev.deviceTypeId == DEV_GROUP:
            del self.groups[dev.address]

        elif dev.deviceTypeId == DEV_TIMECLOCKEVENT:
            del self.events[dev.address]

        elif dev.deviceTypeId == DEV_LINKEDDEVICE:
            pass

        else:
            self.logger.error(u"{}: deviceStopComm: Unknown device type: {}".format(dev.name, dev.deviceTypeId))

    ########################################

    def validateDeviceConfigUi(self, valuesDict, typeId, devId):
        self.logger.debug(u"validateDeviceConfigUi: typeId = {}, devId = {}".format(typeId, devId))

        errorsDict = indigo.Dict()

        if typeId == DEV_KEYPAD and bool(valuesDict[PROP_KEYPADBUT_DISPLAY_LED_STATE]) and int(valuesDict[PROP_KEYPADBUT]) < 80:
            valuesDict[PROP_KEYPADBUT_DISPLAY_LED_STATE] = False
            self.logger.debug(u"validateDeviceConfigUi: forced PROP_KEYPADBUT_DISPLAY_LED_STATE to False for keypad # {}, button # {}".format(valuesDict[PROP_INTEGRATION_ID], valuesDict[PROP_KEYPADBUT]))
        
        if len(errorsDict) > 0:
            return (False, valuesDict, errorsDict)

        return (True, valuesDict)
                
    def runConcurrentThread(self):

        if self.pluginPrefs.get(u"queryAtStartup", False):
            self.queryAllDevices()


        try:
            while True:
                        
                for gateway in self.gateways.values():
                    if gateway.connected:
                        self._processCommand(gateway.poll())
                    
                self.sleep(0.1)
                
        except self.StopThread:
            pass

#########################################
# Poll registered devices for status
#########################################
    def queryAllDevices(self):
        for dev in indigo.devices.iter("self"):
            indigo.device.statusRequest(dev)


# plugin configuration validation
    def validatePrefsConfigUi(self, valuesDict):

        if valuesDict["IP"]:
            if valuesDict["ip_address"].count('.') != 3:
                errorDict = indigo.Dict()
                errorDict["ip_address"] = "Please enter an IP address (i.e. 192.168.1.100)"
                return (False, valuesDict, errorDict)

        if valuesDict["IP"]:

            if self.IP == valuesDict["IP"] and self.pluginPrefs["ip_address"] == valuesDict["ip_address"]: # no changes
                self.logger.debug(u"validatePrefsConfigUi: IP Config, no changes")
                valuesDict["configDone"] = True
            else:
                self.logger.debug(u"validatePrefsConfigUi: IP Config changed")
                valuesDict["configDone"] = False
            
        else:  # serial connection

            serialUrl = self.getSerialPortUrl(valuesDict, u"devicePort")

            if self.IP == valuesDict["IP"] and self.serialUrl == serialUrl: # no changes
                self.logger.debug(u"validatePrefsConfigUi: Serial Config, no changes")
                valuesDict["configDone"] = True
            else:
                self.logger.debug(u"validatePrefsConfigUi: Serial Config changed")
                valuesDict["configDone"] = False

        return (True, valuesDict)


    def closedPrefsConfigUi(self, valuesDict, userCancelled):
        if userCancelled:
            self.logger.debug(u"closedPrefsConfigUi: User Cancelled")
            return
            
        logLevel = int(valuesDict.get(u"logLevel", logging.INFO))
        if logLevel != self.logLevel:
            self.logLevel = logLevel
            self.indigo_log_handler.setLevel(self.logLevel)
            self.logger.debug(u"New logLevel = {}".format(self.logLevel))
            
       
        self.IP = valuesDict["IP"]
        self.runstartup = not valuesDict.get("configDone", False)
        self.logger.debug(u"closedPrefsConfigUi: Setting self.runstartup = {}".format(self.runstartup))
        
        return
           

    ########################################

    def _sendCommand(self, cmd, gateway):
    
        self.gateways[gateway].send(cmd)


    def _processCommand(self, cmd):
        if cmd == None:
            return
            
        cmd = cmd.rstrip()
        if len(cmd) > 0:
            if "~OUTPUT" in cmd:
                self._cmdOutputChange(cmd)
            elif "~DEVICE" in cmd:
                self._cmdDeviceChange(cmd)
            elif "~HVAC" in cmd:
                self._cmdHvacChange(cmd)
            elif "~GROUP" in cmd:
                self._cmdGroup(cmd)
            elif "~TIMECLOCK" in cmd:
                self._cmdTimeClock(cmd)
            elif "~MONITORING" in cmd:
                self.logger.debug(u"Main repeater serial interface configured" + cmd)
            elif "~ERROR" in cmd:
                self.logger.debug(u"{} received".format(cmd))
            elif 'GNET' in cmd:
                #command prompt is ready
                self.logger.threaddebug(u"Command prompt received. Device is ready.")
            elif cmd != "!":
                self.logger.debug(u"Unrecognized command: " + cmd)


    def _cmdOutputChange(self,cmd):
        self.logger.threaddebug(u"Received an Output message: " + cmd)
        cmdArray = cmd.split(',')
        id = cmdArray[1]
        action = cmdArray[2]
        
        if action == '1':  # set level
            try:
                level = float(cmdArray[3])
            except:
                self.logger.warning(u": Unable to parse level as float in _cmdOutputChange: " + str(cmdArray[3]))
                return
                
            if id in self.dimmers:
                zone = self.dimmers[id]
                if int(level) == 0:
                    zone.updateStateOnServer(ONOFF, False)
                else:
                    # zone.updateStateOnServer(ONOFF, True) # Setting brightness below also turns on light
                    zone.updateStateOnServer("brightnessLevel", int(level))
                self.logger.debug(u"Received: Dimmer " + zone.name + " level set to " + str(level))
                
            elif id in self.shades:
                shade = self.shades[id]
                if int(level) == 0:
                    shade.updateStateOnServer(ONOFF, False)
                else:
                    # shade.updateStateOnServer(ONOFF, True) # Setting brightness below also turns on shade, since its basically a light device
                    shade.updateStateOnServer("brightnessLevel", int(level))
                self.logger.debug(u"Received: Shade " + shade.name + " opening set to " + str(level))
                
            elif id in self.switches:
                switch = self.switches[id]
                if int(level) == 0:
                    switch.updateStateOnServer(ONOFF, False)
                    self.logger.debug(u"Received: Switch %s %s" % (switch.name, "turned Off"))
                else:
                    switch.updateStateOnServer(ONOFF, True)
                    self.logger.debug(u"Received: Switch %s %s" % (switch.name, "turned On"))
                    
            elif id in self.ccos:
                cco = self.ccos[id]
                ccoType = cco.pluginProps[PROP_CCO_TYPE]
                if ccoType == "sustained":
                    if int(level) == 0:
                     cco.updateStateOnServer(ONOFF, False)
                    else:
                     cco.updateStateOnServer(ONOFF, True)
                if level == 0.0:
                    self.logger.debug(u"Received: CCO %s %s" % (cco.name, "Opened"))
                else:
                    self.logger.debug(u"Received: CCO %s %s" % (cco.name, "Closed"))
                    
            elif id in self.fans:
                fan = self.fans[id]
                if int(level) == 0:
                    # fan.updateStateOnServer(ONOFF, False) # Setting speed also turns it on/off. This is redundant
                    fan.updateStateOnServer(SPEEDINDEX, 0)
                    fan.updateStateOnServer('ActualSpeed', 0)
                elif level < 26.0:
                    # fan.updateStateOnServer(ONOFF, True) # Setting speed also turns it on/off. This is redundant
                    fan.updateStateOnServer(SPEEDINDEX, 1)
                    fan.updateStateOnServer('ActualSpeed', 25)
                elif level < 51.0:
                    # fan.updateStateOnServer(ONOFF, True) # Setting speed also turns it on/off. This is redundant
                    fan.updateStateOnServer(SPEEDINDEX, 2)
                    fan.updateStateOnServer('ActualSpeed', 50)
                elif level < 76.0:
                    # fan.updateStateOnServer(ONOFF, True) # Setting speed also turns it on/off. This is redundant
                    fan.updateStateOnServer(SPEEDINDEX, 2)
                    fan.updateStateOnServer('ActualSpeed', 75)
                else:
                    # fan.updateStateOnServer(ONOFF, True) # Setting speed also turns it on/off. This is redundant
                    fan.updateStateOnServer(SPEEDINDEX, 3)
                    fan.updateStateOnServer('ActualSpeed', 100)
                self.logger.debug(u"{}: Fan speed set to {}".format(fan.name, level))
                
            return
                
        elif action == '2':  # start raising
            self.logger.debug(u"Received Action 2 for Device " + cmd)
            return
        elif action == '3':  # start lowering
            self.logger.debug(u"Received Action 3 for Device " + cmd)
            return
        elif action == '4':  # stop raising/lowering
            self.logger.debug(u"Received Action 4 for Device " + cmd)
            return
        elif action == '5':  # start flash
            self.logger.debug(u"Received Action 5 for Device " + cmd)
            return
        elif action == '6':  # pulse
            self.logger.debug(u"Received Action 6 for Device " + cmd)
            return
        elif action == '29':  # Lutron firmware 7.5 added an undocumented 29 action code; ignore for now
            return
        elif action == '30':  # Lutron firmware ??? added an undocumented 30 action code; ignore for now
            return
        elif action == '32':  # Lutron firmware ??? added an undocumented 32 action code; ignore for now
            return
        else:
            self.logger.warning(u"Received Unknown Action Code: %s" % cmd)
        return

    def _cmdDeviceChange(self,cmd):
        self.logger.threaddebug(u"Received a Device message: " + cmd)

        cmdArray = cmd.split(',')
        id = cmdArray[1]
        button = cmdArray[2]
        action = cmdArray[3]
        if action == '2':               # this is a motion sensor
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

        keypadid = id + "." + button


        if keypadid in self.phantomButtons:
            self.logger.debug(u"Received a phantom button status message: " + cmd)
            dev = self.phantomButtons[keypadid]
            if status == '0':
                dev.updateStateOnServer(ONOFF, False)
            elif status == '1':
                dev.updateStateOnServer(ONOFF, True)

        if keypadid in self.keypads:
            self.logger.debug(u"Received a keypad button/LED status message: " + cmd)
            dev = self.keypads[keypadid]
            if status == '0':
                dev.updateStateOnServer(ONOFF, False)
            elif status == '1':
                dev.updateStateOnServer(ONOFF, True)
            
            if dev.pluginProps[PROP_KEYPADBUT_DISPLAY_LED_STATE]: # Also display this LED state on its corresponding button
            
                keypadid = id + '.' + str(int(button) - 80)         # Convert LED ID to button ID
                if keypadid in self.keypads:
                    keypad = self.keypads[keypadid]
                    self.logger.debug(u"Updating button status with state of LED ({}) for keypadID {}".format(status, keypadid))
                    if int(status) == 0:
                        keypad.updateStateOnServer(ONOFF, False)
                    elif int(status) == 1:
                        keypad.updateStateOnServer(ONOFF, True)
                else:
                    self.logger.error("WARNING: Invalid ID (%s) specified for LED.   Must be ID of button + 80.  Please correct and reload the plugin." % keypadid)
                    self.logger.debug(keypadid)

            if action == '3': # Check for triggers and linked devices
                self.buttonTriggerCheck(id, button)
                

        if keypadid in self.picos:
            self.logger.debug(u"Received a pico button status message: " + cmd)
            dev = self.picos[keypadid]
            if status == '0':
                dev.updateStateOnServer(ONOFF, False)
            elif status == '1':
                dev.updateStateOnServer(ONOFF, True)

            if action == '3': # Check for triggers and linked devices
                self.buttonTriggerCheck(id, button)

        if keypadid in self.ccis:
            self.logger.debug(u"Received a CCI status message: " + cmd)
            dev = self.ccis[keypadid]
            if status == '0':
                dev.updateStateOnServer(ONOFF, False)
                self.logger.info(u"Received: CCI %s %s" % (dev.name, "Opened"))
            elif status == '1':
                dev.updateStateOnServer(ONOFF, True)
                self.logger.info(u"Received: CCI %s %s" % (dev.name, "Closed"))

        if id in self.sensors:
            self.logger.debug(u"Received a sensor status message: " + cmd)
            dev = self.sensors[id]
            if status == '0':
                dev.updateStateOnServer(ONOFF, False)
                self.logger.info(u"Received: Motion Sensor %s %s" % (dev.name, "vacancy detected"))
            elif status == '1':
                dev.updateStateOnServer(ONOFF, True)
                self.logger.info(u"Received: Motion Sensor %s %s" % (dev.name, "motion detected"))

    # IP comm has not yet been tested with _cmdHvacChange().  Currently left as is -vic13
    def _cmdHvacChange(self,cmd):
        self.logger.debug(u"Received an HVAC message: " + cmd)
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
        self.logger.debug(u"Received a TimeClock message: " + cmd)
        cmdArray = cmd.split(',')
        id = cmdArray[1]
        action = cmdArray[2]
        event = cmdArray[3]
        self.eventTriggerCheck(event)

    def _cmdGroup(self,cmd):
        self.logger.debug(u"Received a Group message  " + cmd)
        cmdArray = cmd.split(',')
        id = cmdArray[1]
        action = cmdArray[2]
        status = cmdArray[3]
        self.groupTriggerCheck(id, status)


    ########################################
    # Relay / Dimmer / Shade / CCO / CCI Action callback
    ########################################
    def actionControlDimmerRelay(self, action, dev):

        sendCmd = ""

        ###### TURN ON ######
        if action.deviceAction == indigo.kDeviceAction.TurnOn:
            if dev.deviceTypeId == DEV_PHANTOM_BUTTON:
                integration_id = dev.pluginProps[PROP_INTEGRATION_ID]
                phantom_button = dev.pluginProps[PROP_COMPONENT_ID]
                sendCmd = ("#DEVICE," + str(int(integration_id)) + ","+ str(int(phantom_button)-100) + ",3,") # Press button
                
            elif dev.deviceTypeId == DEV_PICO:
                pico = dev.pluginProps[PROP_INTEGRATION_ID]
                button = dev.pluginProps[PROP_COMPONENT_ID]
                sendCmd = ("#DEVICE," + pico + "," + button + ",3") # Press button
                
            elif dev.deviceTypeId == DEV_KEYPAD:
                keypad = dev.pluginProps[PROP_INTEGRATION_ID]
                keypadButton = dev.pluginProps[PROP_COMPONENT_ID]
                if (int(keypadButton) > 80):
                    sendCmd = ("#DEVICE," + keypad + "," + str(int(keypadButton)) + ",9,1") # Turn on an LED
                else:
                    sendCmd = ("#DEVICE," + keypad + "," + str(int(keypadButton)) + ",3") # Press button
                    
            elif dev.deviceTypeId == DEV_DIMMER:
                zone = dev.pluginProps[PROP_INTEGRATION_ID]
                sendCmd = ("#OUTPUT," + zone + ",1,100")
                self.lastBrightness[zone] = 100
                
            elif dev.deviceTypeId == DEV_SHADE:
                shade = dev.pluginProps[PROP_INTEGRATION_ID]
                sendCmd = ("#OUTPUT," + shade + ",1,100")
                self.lastBrightness[shade] = 100
                
            elif dev.deviceTypeId == DEV_SWITCH:
                switch = dev.pluginProps[PROP_INTEGRATION_ID]
                sendCmd = ("#OUTPUT," + switch + ",1,100")
                
            elif dev.deviceTypeId == DEV_CCI:
                cci = dev.pluginProps[PROP_INTEGRATION_ID]
                component = dev.pluginProps[PROP_COMPONENT_ID]
                sendCmd = ("#DEVICE," + cci +"," + str(int(component)) + ",3")
                
            elif dev.deviceTypeId == DEV_CCO:
                cco = dev.pluginProps[PROP_INTEGRATION_ID]
                ccoType = dev.pluginProps[PROP_CCO_TYPE]
                if ccoType == "momentary":
                    sendCmd = ("#OUTPUT," + cco + ",6")
                    sendCmd = ("#OUTPUT," + cco + ",1,1")
                else:
                    sendCmd = ("#OUTPUT," + cco + ",1,1")
            
        ###### TURN OFF ######
        elif action.deviceAction == indigo.kDeviceAction.TurnOff:
            if dev.deviceTypeId == DEV_PHANTOM_BUTTON:
                phantom_button = dev.pluginProps[PROP_COMPONENT_ID]
                integration_id = dev.pluginProps[PROP_INTEGRATION_ID]
                sendCmd = ("#DEVICE," + str(int(integration_id)) + ","+ str(int(phantom_button)-100) + ",4,") # Release button
                
            elif dev.deviceTypeId == DEV_PICO:
                pico = dev.pluginProps[PROP_INTEGRATION_ID]
                button = dev.pluginProps[PROP_COMPONENT_ID]
                sendCmd = ("#DEVICE," + pico + "," + button + ",4") # Release button
                
            elif dev.deviceTypeId == DEV_KEYPAD:
                keypad = dev.pluginProps[PROP_INTEGRATION_ID]
                keypadButton = dev.pluginProps[PROP_COMPONENT_ID]
                if (int(keypadButton) > 80):
                    sendCmd = ("#DEVICE," + keypad + "," + str(int(keypadButton)) + ",9,0") # Turn off an LED
                else:
                    sendCmd = ("#DEVICE," + keypad + "," + str(int(keypadButton)) + ",4") # Release button
                    
            elif dev.deviceTypeId == DEV_DIMMER:
                zone = dev.pluginProps[PROP_INTEGRATION_ID]
                sendCmd = ("#OUTPUT," + zone + ",1,0")
                self.lastBrightness[zone] = 0
                
            elif dev.deviceTypeId == DEV_SHADE:
                shade = dev.pluginProps[PROP_INTEGRATION_ID]
                sendCmd = ("#OUTPUT," + shade + ",1,0")
                self.lastBrightness[shade] = 0
                
            elif dev.deviceTypeId == DEV_SWITCH:
                switch = dev.pluginProps[PROP_INTEGRATION_ID]
                sendCmd = ("#OUTPUT," + switch + ",1,0")
                
            elif dev.deviceTypeId == DEV_CCI:
                self.logger.debug(u"it is a cci")
                cci = dev.pluginProps[PROP_INTEGRATION_ID]
                component = dev.pluginProps[PROP_COMPONENT_ID]
                sendCmd = ("#DEVICE," + cci +"," + str(int(component)) + ",4")
                
            elif dev.deviceTypeId == DEV_CCO:
                cco = dev.pluginProps[PROP_INTEGRATION_ID]
                ccoType = dev.pluginProps[PROP_CCO_TYPE]
                if ccoType == "momentary":
                    sendCmd = ("#OUTPUT," + cco + ",6")
                else:
                    sendCmd = ("#OUTPUT," + cco + ",1,0")

        ###### TOGGLE ######
        elif action.deviceAction == indigo.kDeviceAction.Toggle:
            if dev.deviceTypeId == DEV_PHANTOM_BUTTON:
                integration_id = dev.pluginProps[PROP_INTEGRATION_ID]
                phantom_button = dev.pluginProps[PROP_COMPONENT_ID]
                sendCmd = ("#DEVICE," + str(int(integration_id)) + ","+ str(int(phantom_button)-100) + ",3,")
                
            elif dev.deviceTypeId == DEV_KEYPAD:
                keypad = dev.pluginProps[PROP_INTEGRATION_ID]
                keypadButton = dev.pluginProps[PROP_COMPONENT_ID]
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
                        
            elif dev.deviceTypeId == DEV_DIMMER:
                zone = dev.pluginProps[PROP_INTEGRATION_ID]
                if dev.brightness > 0:
                    sendCmd = ("#OUTPUT," + zone + ",1,0")
                else:
                    sendCmd = ("#OUTPUT," + zone + ",1,100")
                    
            elif dev.deviceTypeId == DEV_SHADE:
                shade = dev.pluginProps[PROP_INTEGRATION_ID]
                if dev.brightness > 0:
                    sendCmd = ("#OUTPUT," + shade + ",1,0")
                else:
                    sendCmd = ("#OUTPUT," + shade + ",1,100")
                    
            elif dev.deviceTypeId == DEV_SWITCH:
                switch = dev.pluginProps[PROP_INTEGRATION_ID]
                if dev.onState == True:
                    sendCmd = ("#OUTPUT," + switch + ",1,0")
                else:
                    sendCmd = ("#OUTPUT," + switch + ",1,100")
                    
            elif dev.deviceTypeId == DEV_CCI:
                self.logger.debug(u"it is a cci")
                cci = dev.pluginProps[PROP_INTEGRATION_ID]
                component = dev.pluginProps[PROP_COMPONENT_ID]
                if dev.onState == True:
                    sendCmd = ("#DEVICE," + cci +"," + str(int(component)) + ",4")
                else:
                    sendCmd = ("#DEVICE," + cci +"," + str(int(component)) + ",3")
                    
            elif dev.deviceTypeId == DEV_CCO:
                cco = dev.pluginProps[PROP_INTEGRATION_ID]
                ccoType = dev.pluginProps[PROP_CCO_TYPE]
                if ccoType == "momentary":
                    sendCmd = ("#OUTPUT," + cco + ",6")
                    sendCmd = ("#OUTPUT," + cco + ",1,1")
                else:
                    if dev.onState == True:
                        sendCmd = ("#OUTPUT," + cco + ",1,0")
                    else:
                        sendCmd = ("#OUTPUT," + cco + ",1,1")
                
        ###### SET BRIGHTNESS ######
        elif action.deviceAction == indigo.kDeviceAction.SetBrightness:
            if (dev.deviceTypeId == DEV_DIMMER) or (dev.deviceTypeId == DEV_SHADE):
                newBrightness = action.actionValue
                zone = dev.pluginProps[PROP_INTEGRATION_ID]
                sendCmd = ("#OUTPUT," + dev.pluginProps[PROP_INTEGRATION_ID] + ",1," + str(newBrightness))

        ###### BRIGHTEN BY ######
        elif action.deviceAction == indigo.kDimmerRelayAction.BrightenBy:
            if (dev.deviceTypeId == DEV_DIMMER) or (dev.deviceTypeId == DEV_SHADE):
                newBrightness = dev.brightness + action.actionValue
                if newBrightness > 100:
                    newBrightness = 100
                sendCmd = ("#OUTPUT," + dev.pluginProps[PROP_INTEGRATION_ID] + ",1," + str(newBrightness))

        ###### DIM BY ######
        elif action.deviceAction == indigo.kDimmerRelayAction.DimBy:
            if (dev.deviceTypeId == DEV_DIMMER) or (dev.deviceTypeId == DEV_SHADE):
                newBrightness = dev.brightness - action.actionValue
                if newBrightness < 0:
                    newBrightness = 0
                sendCmd = ("#OUTPUT," + dev.pluginProps[PROP_INTEGRATION_ID] + ",1," + str(newBrightness))

        ###### STATUS REQUEST ######
        elif action.deviceAction == indigo.kDeviceAction.RequestStatus:
            if dev.deviceTypeId == DEV_PHANTOM_BUTTON:
                integration_id = dev.pluginProps[PROP_INTEGRATION_ID]
                phantom_button = dev.pluginProps[PROP_COMPONENT_ID]
                sendCmd = ("?DEVICE," + str(int(integration_id)) + ","+ str(int(phantom_button)) + ",9,")
                
            elif dev.deviceTypeId == DEV_KEYPAD:
                integration_id = dev.pluginProps[PROP_INTEGRATION_ID]
                keypadButton = dev.pluginProps[PROP_COMPONENT_ID]
                if (int(keypadButton) > 80):
                    sendCmd = ("?DEVICE," + integration_id + "," + str(int(keypadButton)) + ",9")
                else:
                    sendCmd = ("?DEVICE," + integration_id + "," + str(int(keypadButton)+80) + ",9")
                    
            elif (dev.deviceTypeId == DEV_DIMMER) or (dev.deviceTypeId == DEV_SHADE) or (dev.deviceTypeId == DEV_SWITCH):
                integration_id = dev.pluginProps[PROP_INTEGRATION_ID]
                sendCmd = ("?OUTPUT," + integration_id + ",1,")
                                
            elif dev.deviceTypeId == DEV_CCO:
                cco = dev.pluginProps[PROP_INTEGRATION_ID]
                ccoType = dev.pluginProps[PROP_CCO_TYPE]
                if ccoType == "momentary":
                    self.logger.info(u"Momentary CCOs do not respond to Status Requests")
                else:
                    sendCmd = ("?OUTPUT," + cco + ",1,")

            elif dev.deviceTypeId == DEV_CCI:
                self.logger.info(u"This device does not respond to Status Requests")

        if len(sendCmd):
            gateway = dev.pluginProps['gateway']
            self._sendCommand(sendCmd, gateway)
            self.logger.debug(u"{}: actionControlDimmerRelay sent: '{}' to gateway {}".format(dev.name, sendCmd, gateway))

    ######################
    # Sensor Action callback
    ######################
    def actionControlSensor(self, action, dev):
        self.logger.debug(u"{}: This device does not respond to Status Requests".format(dev.name))

    ######################
    # Fan Action callback
    ######################
    def actionControlSpeedControl(self, action, dev):
        
        ###### TURN ON ######
        if action.speedControlAction == indigo.kSpeedControlAction.TurnOn:
            self.logger.debug(u"{}: TurnOn".format(dev.name))

            if dev.deviceTypeId == DEV_FAN:
                sendCmd = "#OUTPUT,{},1,{}".format(dev.pluginProps[PROP_INTEGRATION_ID], dev.pluginProps[PROP_LASTSPEED])
                dev.updateStateOnServer(ACTUALSPEED, int(dev.pluginProps[PROP_LASTSPEED]))
                dev.updateStateOnServer(ONOFF, True)
                dev.updateStateOnServer(SPEEDINDEX, dev.pluginProps[PROP_LASTSPEED])

        ###### TURN OFF ######
        elif action.speedControlAction == indigo.kSpeedControlAction.TurnOff:
            self.logger.debug(u"{}: TurnOff".format(dev.name))

            if dev.deviceTypeId == DEV_FAN:
                sendCmd = "#OUTPUT,{},1,0".format(dev.pluginProps[PROP_INTEGRATION_ID])
                if dev.states[ACTUALSPEED] > 0:
                    self.update_device_property(dev, PROP_LASTSPEED, dev.states[ACTUALSPEED])
                dev.updateStateOnServer(ACTUALSPEED, 0)
                dev.updateStateOnServer(ONOFF, False)
                dev.updateStateOnServer(SPEEDINDEX, "0")

        ###### TOGGLE ######
        elif action.speedControlAction == indigo.kSpeedControlAction.Toggle:
            self.logger.debug(u"{}: Toggle".format(dev.name))

            if dev.deviceTypeId == DEV_FAN:
            
                if int(dev.states[ACTUALSPEED]) > 0:      # turn Off
                    sendCmd = "#OUTPUT,{},1,0".format(dev.pluginProps[PROP_INTEGRATION_ID])
                    if dev.states[ACTUALSPEED] > 0:
                        self.update_device_property(dev, PROP_LASTSPEED, dev.states[ACTUALSPEED])
                    dev.updateStateOnServer(ACTUALSPEED, 0)
                    dev.updateStateOnServer(ONOFF, False)
                    dev.updateStateOnServer(SPEEDINDEX, "0")
            
                else:  
                    sendCmd = "#OUTPUT,{},1,{}".format(dev.pluginProps[PROP_INTEGRATION_ID], dev.pluginProps[PROP_LASTSPEED])
                    dev.updateStateOnServer(ACTUALSPEED, int(dev.pluginProps[PROP_LASTSPEED]))
                    dev.updateStateOnServer(ONOFF, True)
                    dev.updateStateOnServer(SPEEDINDEX, dev.pluginProps[PROP_LASTSPEED])
            

        ###### SET SPEED INDEX ######
        elif action.speedControlAction == indigo.kSpeedControlAction.SetSpeedIndex:
            self.logger.debug(u"{}: SetSpeedIndex to {}".format(dev.name, action.actionValue))

            if dev.deviceTypeId == DEV_FAN:
                newSpeedIndex = action.actionValue
                if newSpeedIndex == 0:
                    sendCmd = "#OUTPUT,{},1,0".format(dev.pluginProps[PROP_INTEGRATION_ID])
                    dev.updateStateOnServer(ONOFF, False)
                    dev.updateStateOnServer(SPEEDINDEX, 0)
                    dev.updateStateOnServer(ACTUALSPEED, 0)
                elif newSpeedIndex == 1:
                    sendCmd = "#OUTPUT,{},1,25".format(dev.pluginProps[PROP_INTEGRATION_ID])
                    self.update_device_property(dev, PROP_LASTSPEED, "25")
                    dev.updateStateOnServer(ONOFF, True)
                    dev.updateStateOnServer(SPEEDINDEX, 1)
                    dev.updateStateOnServer(ACTUALSPEED, 25)
                elif newSpeedIndex == 2:
                    sendCmd = "#OUTPUT,{},1,75".format(dev.pluginProps[PROP_INTEGRATION_ID])
                    self.update_device_property(dev, PROP_LASTSPEED, "75")
                    dev.updateStateOnServer(ONOFF, True)
                    dev.updateStateOnServer(SPEEDINDEX, 2)
                    dev.updateStateOnServer(ACTUALSPEED, 75)
                elif newSpeedIndex == 3:
                    sendCmd = "#OUTPUT,{},1,100".format(dev.pluginProps[PROP_INTEGRATION_ID])
                    self.update_device_property(dev, PROP_LASTSPEED, "100")
                    dev.updateStateOnServer(ONOFF, True)
                    dev.updateStateOnServer(SPEEDINDEX, 3)
                    dev.updateStateOnServer(ACTUALSPEED, 100)
                else:
                    self.logger.error(u"{}: Invalid speedIndex = {}".format(dev.name, newSpeed))


        ###### SET SPEED LEVEL ######
        elif action.speedControlAction == indigo.kSpeedControlAction.SetSpeedLevel:
            self.logger.debug(u"{}: SetSpeedLevel to {}".format(dev.name, action.actionValue))

            if dev.deviceTypeId == DEV_FAN:
                newSpeedLevel = int(action.actionValue)
                sendCmd = "#OUTPUT,{},1,{}".format(dev.pluginProps[PROP_INTEGRATION_ID], action.actionValue)
                dev.updateStateOnServer(ACTUALSPEED, action.actionValue)
                if dev.states[ACTUALSPEED] > 0:
                    self.update_device_property(dev, PROP_LASTSPEED, dev.states[ACTUALSPEED])

                if newSpeedLevel == 0:
                    dev.updateStateOnServer(ONOFF, False)
                    dev.updateStateOnServer(SPEEDINDEX, 0)
                elif newSpeedLevel < 26:
                    dev.updateStateOnServer(ONOFF, True)
                    dev.updateStateOnServer(SPEEDINDEX, 1)
                elif newSpeedLevel < 76:
                    dev.updateStateOnServer(ONOFF, True)
                    dev.updateStateOnServer(SPEEDINDEX, 2)
                else:
                    dev.updateStateOnServer(ONOFF, True)
                    dev.updateStateOnServer(SPEEDINDEX, 3)


        ###### INCREASE SPEED INDEX BY ######
        elif action.speedControlAction == indigo.kSpeedControlAction.IncreaseSpeedIndex:
            self.logger.debug(u"{}: IncreaseSpeedIndex by {}".format(dev.name, action.actionValue))

            if dev.deviceTypeId == DEV_FAN:
                newSpeedIndex = dev.speedIndex + action.actionValue
                if newSpeedIndex > 3:
                    newSpeedIndex = 3
                
                if newSpeedIndex == 0:
                    sendCmd = "#OUTPUT,{},1,0".format(dev.pluginProps[PROP_INTEGRATION_ID])
                    self.update_device_property(dev, PROP_LASTSPEED, dev.states[ACTUALSPEED])
                    dev.updateStateOnServer(ONOFF, False)
                    dev.updateStateOnServer(SPEEDINDEX, 0)
                elif newSpeedIndex == 1:
                    sendCmd = "#OUTPUT,{},1,25".format(dev.pluginProps[PROP_INTEGRATION_ID])
                    self.update_device_property(dev, PROP_LASTSPEED, "25")
                    dev.updateStateOnServer(ONOFF, True)
                    dev.updateStateOnServer(SPEEDINDEX, 1)
                    dev.updateStateOnServer(ACTUALSPEED, 25)
                elif newSpeedIndex == 2:
                    sendCmd = "#OUTPUT,{},1,75".format(dev.pluginProps[PROP_INTEGRATION_ID])
                    self.update_device_property(dev, PROP_LASTSPEED, "75")
                    dev.updateStateOnServer(ONOFF, True)
                    dev.updateStateOnServer(SPEEDINDEX, 2)
                    dev.updateStateOnServer(ACTUALSPEED, 75)
                elif newSpeedIndex == 3:
                    sendCmd = "#OUTPUT,{},1,100".format(dev.pluginProps[PROP_INTEGRATION_ID])
                    self.update_device_property(dev, PROP_LASTSPEED, "100")
                    dev.updateStateOnServer(ONOFF, True)
                    dev.updateStateOnServer(SPEEDINDEX, 3)
                    dev.updateStateOnServer(ACTUALSPEED, 100)
                else:
                    self.logger.error(u"{}: Invalid speedIndex = {}".format(dev.name, newSpeed))


        ###### DECREASE SPEED INDEX BY ######
        elif action.speedControlAction == indigo.kSpeedControlAction.DecreaseSpeedIndex:
            self.logger.debug(u"{}: DecreaseSpeedIndex by {}".format(dev.name, action.actionValue))

            if dev.deviceTypeId == DEV_FAN:
                newSpeedIndex = dev.speedIndex - action.actionValue
                if newSpeedIndex < 0:
                    newSpeedIndex = 0

                if newSpeedIndex == 0:
                    sendCmd = "#OUTPUT,{},1,0".format(dev.pluginProps[PROP_INTEGRATION_ID])
                    self.update_device_property(dev, PROP_LASTSPEED, dev.states[ACTUALSPEED])
                    dev.updateStateOnServer(ONOFF, False)
                    dev.updateStateOnServer(SPEEDINDEX, 0)
                elif newSpeedIndex == 1:
                    sendCmd = "#OUTPUT,{},1,25".format(dev.pluginProps[PROP_INTEGRATION_ID])
                    self.update_device_property(dev, PROP_LASTSPEED, "25")
                    dev.updateStateOnServer(ONOFF, True)
                    dev.updateStateOnServer(SPEEDINDEX, 1)
                    dev.updateStateOnServer(ACTUALSPEED, 25)
                elif newSpeedIndex == 2:
                    sendCmd = "#OUTPUT,{},1,75".format(dev.pluginProps[PROP_INTEGRATION_ID])
                    self.update_device_property(dev, PROP_LASTSPEED, "75")
                    dev.updateStateOnServer(ONOFF, True)
                    dev.updateStateOnServer(SPEEDINDEX, 2)
                    dev.updateStateOnServer(ACTUALSPEED, 75)
                elif newSpeedIndex == 3:
                    sendCmd = "#OUTPUT,{},1,100".format(dev.pluginProps[PROP_INTEGRATION_ID])
                    self.update_device_property(dev, PROP_LASTSPEED, "100")
                    dev.updateStateOnServer(ONOFF, True)
                    dev.updateStateOnServer(SPEEDINDEX, 3)
                    dev.updateStateOnServer(ACTUALSPEED, 100)
                else:
                    self.logger.error(u"{}: Invalid speedIndex = {}".format(dev.name, newSpeed))


        ###### STATUS REQUEST ######
        elif action.speedControlAction == indigo.kUniversalAction.RequestStatus:
            integration_id = dev.pluginProps[PROP_INTEGRATION_ID]
            sendCmd = "?OUTPUT,{},1,".format(integration_id)

        if len(sendCmd):
            gateway = dev.pluginProps['gateway']
            self._sendCommand(sendCmd, gateway)
            self.logger.debug(u"{}: actionControlSpeedControl sent: '{}' to gateway {}".format(dev.name, sendCmd, gateway))


    ######################
    # HVAC Action callback
    ######################

    def actionControlThermostat(self, action, dev):

        sendCmd = ""

        integration_id = dev.pluginProps[PROP_INTEGRATION_ID]
        currentCoolSetpoint = dev.coolSetpoint
        currentHeatSetpoint = dev.heatSetpoint

        ###### SET SETPOINTS ######
        if action.thermostatAction == indigo.kThermostatAction.DecreaseCoolSetpoint:
            newCoolSetpoint = float(currentCoolSetpoint) - 1
            newHeatSetpoint = float(currentHeatSetpoint)
            sendCmd = "#HVAC," + integration_id + ",2," + str(newHeatSetpoint) + "," + str(newCoolSetpoint)
            
        elif action.thermostatAction == indigo.kThermostatAction.IncreaseCoolSetpoint:
            newCoolSetpoint = float(currentCoolSetpoint) + 1
            newHeatSetpoint = float(currentHeatSetpoint)
            sendCmd = "#HVAC," + integration_id + ",2," + str(newHeatSetpoint) + "," + str(newCoolSetpoint)
            
        elif action.thermostatAction == indigo.kThermostatAction.DecreaseHeatSetpoint:
            newCoolSetpoint = float(currentCoolSetpoint)
            newHeatSetpoint = float(currentHeatSetpoint) - 1
            sendCmd = "#HVAC," + integration_id + ",2," + str(newHeatSetpoint) + "," + str(newCoolSetpoint)
            
        elif action.thermostatAction == indigo.kThermostatAction.IncreaseHeatSetpoint:
            newCoolSetpoint = float(currentCoolSetpoint)
            newHeatSetpoint = float(currentHeatSetpoint) + 1
            sendCmd = "#HVAC," + integration_id + ",2," + str(newHeatSetpoint) + "," + str(newCoolSetpoint)
            
        elif action.thermostatAction == indigo.kThermostatAction.SetHeatSetpoint:
            newCoolSetpoint = float(currentCoolSetpoint)
            newHeatSetpoint = action.actionValue
            dev.updateStateOnServer("setpointHeat", newHeatSetpoint)
            sendCmd = "#HVAC," + integration_id + ",2," + str(newHeatSetpoint) + "," + str(newCoolSetpoint) +"\r"
            
        elif action.thermostatAction == indigo.kThermostatAction.SetCoolSetpoint:
            newCoolSetpoint = action.actionValue
            dev.updateStateOnServer("setpointCool", newCoolSetpoint)
            newHeatSetpoint = float(currentHeatSetpoint)
            sendCmd = "#HVAC," + integration_id + ",2," + str(newHeatSetpoint) + "," + str(newCoolSetpoint) +"\r"

        ###### SET HVAC MODE ######
        elif action.thermostatAction == indigo.kThermostatAction.SetHvacMode:
            mode = action.actionMode
            if mode == indigo.kHvacMode.Off:
                sendCmd = "#HVAC," + integration_id + ",3,1"
            elif mode == indigo.kHvacMode.Heat:
                sendCmd = "#HVAC," + integration_id + ",3,2"
            elif mode == indigo.kHvacMode.Cool:
                sendCmd = "#HVAC," + integration_id + ",3,3"
            elif mode == indigo.kHvacMode.HeatCool:
                sendCmd = "#HVAC," + integration_id + ",3,4"

        ###### SET FAN MODE ######
        elif action.thermostatAction == indigo.kThermostatAction.SetFanMode:
            mode = action.actionMode
            if mode == indigo.kFanMode.Auto:
                sendCmd = "#HVAC," + integration_id + ",4,1"
            elif mode == indigo.kFanMode.AlwaysOn:
                sendCmd = "#HVAC," + integration_id + ",4,2"

        ###### STATUS REQUEST ######
        elif action.thermostatAction == indigo.kThermostatAction.RequestStatusAll:

            gateway = dev.pluginProps['gateway']

            sendCmd = "?HVAC," + integration_id + ",1," # get temperature
            self._sendCommand(sendCmd, gateway)
            self.logger.debug(u"{}: actionControlSpeedControl sent: '{}' to gateway {}".format(dev.name, sendCmd, gateway))
            
            sendCmd = "?HVAC," + integration_id + ",2," # get heat and cool setpoints
            self._sendCommand(sendCmd, gateway)
            self.logger.debug(u"{}: actionControlSpeedControl sent: '{}' to gateway {}".format(dev.name, sendCmd, gateway))
            
            sendCmd = "?HVAC," + integration_id + ",3," # get operating mode
            self._sendCommand(sendCmd, gateway)
            self.logger.debug(u"{}: actionControlSpeedControl sent: '{}' to gateway {}".format(dev.name, sendCmd, gateway))

            sendCmd = "?HVAC," + integration_id + ",4," # get fan mode
            self._sendCommand(sendCmd, gateway)
            self.logger.debug(u"{}: actionControlSpeedControl sent: '{}' to gateway {}".format(dev.name, sendCmd, gateway))
            return
            
        # only if not request status, which sends multiple commmands 
        if len(sendCmd):
            gateway = dev.pluginProps['gateway']
            self._sendCommand(sendCmd, gateway)
            self.logger.debug(u"{}: actionControlSpeedControl sent: '{}' to gateway {}".format(dev.name, sendCmd, gateway))

    ########################################
    # Plugin Actions object callbacks (pluginAction is an Indigo plugin action instance)

    def setFanSpeed(self, pluginAction, dev):

        gateway = dev.pluginProps['gateway']

        fanSpeed =  pluginAction.props["fanSpeed"]
        sendCmd = "#OUTPUT,{},1,{}".format(dev.address, fanSpeed)
        self.logger.debug(u"{}: Sending set fan speed {} to {}".format(dev.name, fanSpeed, gateway))
        self._sendCommand(sendCmd, gateway)

    def fadeDimmer(self, pluginAction, dev):

        gateway = dev.pluginProps['gateway']

        brightness =  indigo.activePlugin.substitute(pluginAction.props["brightness"])
        fadeTime =  indigo.activePlugin.substitute(pluginAction.props["fadeTime"])
        zone = dev.address

        sendCmd = ("#OUTPUT," + zone + ",1," + str(brightness) + "," + str(fadeTime))
        self.logger.debug(u"{}: Sending set brightness {} with fade {} to {}".format(dev.name, brightness, fadeTime, gateway))
        self._sendCommand(sendCmd, gateway)

    def startRaising(self, pluginAction, dev):

        gateway = dev.pluginProps['gateway']

        zone = shadeDevice.address

        sendCmd = ("#OUTPUT," + zone + ",2")
        self.logger.info(u"{}: Start Raising".format(dev.name))
        self._sendCommand(sendCmd, gateway)

    def startLowering(self, pluginAction, dev):

        gateway = dev.pluginProps['gateway']

        zone = shadeDevice.address

        sendCmd = ("#OUTPUT," + zone + ",3")
        self.logger.info(u"{}: Start Lowering".format(dev.name))
        self._sendCommand(sendCmd, gateway)

    def stopRaiseLower(self, pluginAction, dev):

        gateway = dev.pluginProps['gateway']

        zone = dev.address

        sendCmd = ("#OUTPUT," + zone + ",4")
        self.logger.info(u"{}: Stop Raising/Lowering".format(dev.name))
        self._sendCommand(sendCmd, gateway)

    def sendRawCommand(self, pluginAction):

        gateway = pluginAction.pluginProps['gateway']
        sendCmd =  indigo.activePlugin.substitute(pluginAction.props["commandString"])
        self.logger.debug(u"Sending Raw Command: '{}'".formatsendCmd)
        self._sendCommand(sendCmd, gateway)
        
    ########################################

    def get_gateway_list(self, filter="", valuesDict=None, typeId="", targetId=0):
        self.logger.threaddebug("get_gateway_list: typeId = {}, targetId = {}, valuesDict = {}".format(typeId, targetId, valuesDict))
        gateways = [
            (gateway.dev.id, indigo.devices[gateway.dev.id].name)
            for gateway in self.gateways.values()
        ]
        return gateways
        

    def roomListGenerator(self, filter=None, valuesDict=None, typeId=0, targetId=0):
        self.logger.threaddebug(u"roomListGenerator, typeId = {}, targetId = {}, valuesDict = {}".format(typeId, targetId, valuesDict))
        retList = []
        for room in self.roomButtonTree:
            self.logger.threaddebug(u"roomListGenerator adding: {} {}".format(room, room))         
            retList.append((room, room))
        
        retList.sort(key=lambda tup: tup[1])
        return retList

    def pickKeypadButton(self, filter=None, valuesDict=None, typeId=0, targetId=0):
        self.logger.threaddebug(u"pickKeypadButton, typeId = {}, targetId = {}, valuesDict = {}".format(typeId, targetId, valuesDict))
        retList = []
        try:
            room = valuesDict["room"]
        except:
            return retList
        if len(room) == 0:
            return retList
            
        for buttonId in self.roomButtonTree[room]:
            self.logger.threaddebug(u"pickKeypadButton adding: {} ({})".format(buttonId, self.roomButtonTree[room][buttonId]))         
            retList.append((buttonId, self.roomButtonTree[room][buttonId]))
         
        retList.sort(key=lambda tup: tup[1])
        return retList

    def pickEvent(self, filter=None, valuesDict=None, typeId=0, targetId=0):
        self.logger.threaddebug(u"pickEvent, typeId = {}, targetId = {}, valuesDict = {}".format(typeId, targetId, valuesDict))
        retList = []
        for dev in indigo.devices.iter("self.ra2TimeClockEvent"):
            event = dev.pluginProps[PROP_EVENT]
            self.logger.threaddebug(u"pickEvent adding: {}".format(event))         
            retList.append((event, dev.name))
        retList.sort(key=lambda tup: tup[1])
        return retList

    def pickGroup(self, filter=None, valuesDict=None, typeId=0, targetId=0):
        self.logger.threaddebug(u"pickGroup, typeId = {}, targetId = {}, valuesDict = {}".format(typeId, targetId, valuesDict))
        retList = []
        for dev in indigo.devices.iter("self.ra2Group"):
            group = dev.pluginProps[PROP_GROUP]
            self.logger.threaddebug(u"pickGroup adding: {}".format(group))         
            retList.append((group, dev.name))
        retList.sort(key=lambda tup: tup[1])
        return retList

    def controllableDevices(self, filter="", valuesDict=None, typeId="", targetId=0):
        self.logger.threaddebug(u"controllableDevices, typeId = {}, targetId = {}, valuesDict = {}".format(typeId, targetId, valuesDict))
        retList = []
        for dev in indigo.devices:
            if hasattr(dev, "onState"):
                if dev.pluginId != "com.jimandnoreen.indigoplugin.lutron-radiora2":
                    retList.append((dev.id, dev.name))
        retList.sort(key=lambda tup: tup[1])
        return retList

    def menuChanged(self, valuesDict, typeId=None, devId=None):
        self.logger.threaddebug(u"menuChanged, typeId = {}, valuesDict = {}".format(typeId, valuesDict))
        return valuesDict

    ########################################
    # This is the method that's called by the Add Linked Device button in the config dialog.
    ########################################
    def addLinkedDevice(self, valuesDict, typeId=None, devId=None):
        
        buttonDeviceId = valuesDict["buttonDevice"]
        controlledDeviceId = valuesDict["controlledDevice"]
        linkName = valuesDict["linkName"]

        try:
            buttonDevice = indigo.devices[int(buttonDeviceId)]
            buttonAddress = buttonDevice.address
        except:
            self.logger.debug(u"addLinkedDevice: buttonDevice not found: {}".format(valuesDict['buttonDevice']))
            return

        parts = buttonAddress.split(".")
        deviceID =  parts[0]
        componentID = parts[1]
        buttonLEDAddress = "{}.{}".format(deviceID, int(componentID)+80)
        try:
            buttonLEDDeviceId = unicode(self.keypads[buttonLEDAddress].id)
        except:
            buttonLEDDeviceId = "0"
        linkID = "{}-{}".format(buttonDeviceId, controlledDeviceId)
        if len(linkName) == 0: 
            linkName = linkID
        linkItem = {"name" : linkName, "buttonDevice" : buttonDeviceId, "buttonLEDDevice" : buttonLEDDeviceId, "controlledDevice" : controlledDeviceId, "buttonAddress" : buttonAddress}
        self.logger.debug(u"Adding linkItem {}: {}".format(linkID, linkItem))
        self.linkedDeviceList[linkID] = linkItem
        self.logLinkedDevices()
        
        indigo.activePlugin.pluginPrefs[u"linkedDevices"] = json.dumps(self.linkedDeviceList)

    ########################################
    # This is the method that's called by the Delete Device button
    ########################################
    def deleteLinkedDevices(self, valuesDict, typeId=None, devId=None):
        
        for item in valuesDict["linkedDeviceList"]:
            self.logger.info(u"deleting device {}".format(item))
            del self.linkedDeviceList[item]

        self.logLinkedDevices()
        indigo.activePlugin.pluginPrefs[u"linkedDevices"] = json.dumps(self.linkedDeviceList)
        
    def listLinkedDevices(self, filter="", valuesDict=None, typeId="", targetId=0):
        returnList = list()
        for linkID, linkItem in self.linkedDeviceList.iteritems():
            returnList.append((linkID, linkItem["name"]))
        return sorted(returnList, key= lambda item: item[1])

    ########################################
    
    def logLinkedDevices(self):
        if len(self.linkedDeviceList) == 0:
            self.logger.info(u"No linked Devices")
            return
            
        fstring = u"{:^25} {:^25} {:^20} {:^20} {:^20} {:^20}"
        self.logger.info(fstring.format("Link ID", "Link Name", "buttonDevice", "buttonLEDDevice", "controlledDevice", "buttonAddress"))
        for linkID, linkItem in self.linkedDeviceList.iteritems():
             self.logger.info(fstring.format(linkID, linkItem["name"], linkItem["buttonDevice"], linkItem["buttonLEDDevice"], linkItem["controlledDevice"], linkItem["buttonAddress"]))
            
    ########################################

    def createCasetaDevicesMenu(self, valuesDict, typeId):

        deviceThread = threading.Thread(target = self.createCasetaDevices, args = (valuesDict, ))
        deviceThread.start()    
        return True        

    def createCasetaDevices(self, valuesDict):

        if not self.threadLock.acquire(False):
            self.logger.warning(u"Unable to create devices, process already running.")
            return

        gatewayID = valuesDict["gateway"]

        self.group_by = valuesDict["group_by"]
        self.create_bridge_buttons = valuesDict["create_bridge_buttons"]
        self.jsonText = valuesDict["jsonText"]

        self.logger.info(u"Creating Devices from JSON data, Grouping = {}".format(self.group_by))

        casetaData = json.loads(self.jsonText)
        
        for device in casetaData["LIPIdList"]["Devices"]:
            self.logger.info(u"Caseta Device '{}' ({}), Buttons = {}".format(device["Name"], device["ID"], len(device["Buttons"])))
 
            if device["ID"] == 1 and not self.create_bridge_buttons:
                self.logger.debug(u"Skipping Smart Bridge button creation")
                continue
                
            try:
                areaName = device["Area"]["Name"]
            except:
                areaname = u"Bridge"
                
            for button in device["Buttons"]:
            
                address = "{}:{}.{}".format(gatewayID, device["ID"], button["Number"])
                name = u"{} - {} ({})".format(areaName, device["Name"], address)
                props = {
                    PROP_ROOM : areaName, 
                    PROP_LIST_TYPE : "button", 
                    PROP_GATEWAY: gatewayID,
                    PROP_INTEGRATION_ID : str(device["ID"]), 
                    PROP_COMPONENT_ID : str(button["Number"]), 
                    PROP_BUTTONTYPE : "Unknown",
                    PROP_ISBUTTON : "True"
                }
                self.createLutronDevice(DEV_PICO, name, address, props, areaName)                    
           
        for zone in casetaData["LIPIdList"]["Zones"]:
            self.logger.info(u"Caseta Zone '{}' ({}), Area = {}".format(zone["Name"], zone["ID"], zone["Area"]["Name"]))

            try:
                areaName = zone["Area"]["Name"]
            except:
                areaname = u"Unknown"
                
            address = "{}:{}".format(gatewayID, zone["ID"])
            name = u"{} - {} ({})".format(areaName, zone["Name"], zone["ID"])
            props = {
                PROP_ROOM : areaName, 
                PROP_GATEWAY: gatewayID,
                PROP_INTEGRATION_ID : str(zone["ID"]),
                PROP_OUTPUTTYPE: "AUTO_DETECT"
            }
            self.createLutronDevice(DEV_DIMMER, name, address, props, areaName)
   
        self.logger.info(u"Creating Devices done.")        
        self.threadLock.release()
        return


    def createRRA2DevicesMenu(self, valuesDict, typeId):
        
        gateway = self.gateways[int(valuesDict["gateway"])]
        
        if not gateway.connected:
            self.logger.warning(u"Unable to create devices, no connection to repeater.")
            return False
            
        self.logger.debug(u"Starting device fetch thread...")
        deviceThread = threading.Thread(target = self.createRRA2Devices, args = (valuesDict, ))
        deviceThread.start()    
        return True        

    def createRRA2Devices(self, valuesDict):
        
        self.logger.debug(u"Device fetch thread running, valuesDict = {}".format(valuesDict))

        if not self.threadLock.acquire(False):
            self.logger.warning(u"Unable to create devices, process already running.")
            return

        # set up variables based on options selected

        gatewayID = valuesDict["gateway"]
        
        self.group_by = valuesDict["group_by"]
        self.create_unused_keypad = bool(valuesDict["create_unused_keypad"])
        self.create_unused_phantom = bool(valuesDict["create_unused_phantom"])
        self.create_event_triggers = bool(valuesDict["create_event_triggers"])
        self.create_group_triggers = bool(valuesDict["create_group_triggers"])

        if bool(valuesDict["use_local"]):
            self.logger.info(u"Creating Devices from file: %s, Grouping = %s, Create unprogrammed keypad buttons = %s, Create unprogrammed phantom buttons = %s" % \
                (xmlFile, self.group_by, self.create_unused_keypad, self.create_unused_phantom))
            try:
                xmlFile = os.path.expanduser(valuesDict["xmlFileName"])         
                root = ET.parse(xmlFile).getroot()
            except:
                self.logger.error(u"Unable to parse XML file: {}".format(xmlFile))
                self.threadLock.release()
                return
                
            self.logger.info(u"Creating Devices file read completed, parsing data...")

        else:
        
            self.logger.info(u"Creating RRA2 Devices from gateway {}, Grouping = {}, Create unprogrammed keypad buttons = {}, Create unprogrammed phantom buttons = {}".format(gatewayID, self.group_by, self.create_unused_keypad, self.create_unused_phantom))
            self.logger.info(u"Creating Devices - starting data fetch...")
                    
            try:
                text = self.gateways[int(gatewayID)].fetchXML()
                root = ET.fromstring(text)        
            except:
                self.logger.error(u"Unable to parse XML data from repeater.")
                self.threadLock.release()
                return

            self.logger.info(u"Creating Devices fetch completed, parsing data...")
        
        # iterate through parts of the XML data, 'Areas' first
        
        for room in root.findall('Areas/Area/Areas/Area'):
            self.logger.info("Finding devices in '%s'" % (room.attrib['Name']))
        
            for device in room.findall('DeviceGroups/Device'):
                self.logger.debug("Device: %s (%s,%s)" % (device.attrib['Name'], device.attrib['IntegrationID'], device.attrib['DeviceType']))
                if device.attrib['DeviceType'] == "MAIN_REPEATER":
                    for component in device.findall('Components/Component'):
                        self.logger.debug("Component: %s (%s)" % (component.attrib['ComponentNumber'], component.attrib['ComponentType']))
                        if component.attrib['ComponentType'] == "BUTTON":

                            assignments = len(component.findall('Button/Actions/Action/Presets/Preset/PresetAssignments/PresetAssignment'))                            
                            if not self.create_unused_phantom and assignments == 0:
                                continue

                            name = u"Phantom Button {:03}.{:03}".format(int(device.attrib['IntegrationID']), int(component.attrib['ComponentNumber']))
                            try:                          
                                engraving = component.find("Button").attrib['Engraving']
                                name = name + " - " + engraving
                            except:
                                pass
                            button = str(int(component.attrib['ComponentNumber']) + 100)
                            address = gatewayID + ":" + device.attrib['IntegrationID'] + "." + button

                            try:
                                buttonType = component.find("Button").attrib[PROP_BUTTONTYPE]
                            except:
                                buttonType = "Unknown"
                            props = {
                                PROP_ROOM : room.attrib['Name'], 
                                PROP_GATEWAY: gatewayID,
                                PROP_INTEGRATION_ID : device.attrib['IntegrationID'], 
                                PROP_COMPONENT_ID : button, 
                                PROP_BUTTONTYPE : buttonType,
                                PROP_ISBUTTON : "True"
                            }
                            self.createLutronDevice(DEV_PHANTOM_BUTTON, name, address, props, room.attrib['Name'])

                        elif component.attrib['ComponentType'] == "LED":    # ignore LEDs for phantom buttons
                            pass

                        else:
                            self.logger.error("Unknown Component Type: %s (%s)" % (component.attrib['Name'], component.attrib['ComponentType']))

                else:
                    self.logger.error("Unknown Device Type: %s (%s)" % (device.attrib['Name'], device.attrib['DeviceType']))
                    
            for output in room.findall('Outputs/Output'):
                self.logger.debug("Output: %s (%s) %s" % (output.attrib['Name'], output.attrib['IntegrationID'], output.attrib['OutputType']))

                if output.attrib['OutputType'] == "INC" or output.attrib['OutputType'] == "MLV" or output.attrib['OutputType'] == "AUTO_DETECT":
                    address = gatewayID + ":" + output.attrib['IntegrationID']
                    name = u"{} - Dimmer {} - {}".format(room.attrib['Name'], output.attrib['IntegrationID'], output.attrib['Name'])
                    props = {
                        PROP_ROOM : room.attrib['Name'], 
                        PROP_GATEWAY: gatewayID,
                        PROP_INTEGRATION_ID : output.attrib['IntegrationID'],
                        PROP_OUTPUTTYPE: output.attrib[PROP_OUTPUTTYPE],
                    }
                    self.createLutronDevice(DEV_DIMMER, name, address, props, room.attrib['Name'])
                    
                elif output.attrib['OutputType'] == "NON_DIM":
                    address = gatewayID + ":" + output.attrib['IntegrationID']
                    name = u"{} - Switch {} - {}".format(room.attrib['Name'], output.attrib['IntegrationID'], output.attrib['Name'])
                    props = {
                        PROP_ROOM : room.attrib['Name'], 
                        PROP_GATEWAY: gatewayID,
                        PROP_INTEGRATION_ID : output.attrib['IntegrationID'],
                        PROP_OUTPUTTYPE: output.attrib[PROP_OUTPUTTYPE]
                    }
                    self.createLutronDevice(DEV_SWITCH, name, address, props, room.attrib['Name'])
                        
                elif output.attrib['OutputType'] == "SYSTEM_SHADE":
                    address = gatewayID + ":" + output.attrib['IntegrationID']
                    name = u"{} - Shade {} - {}".format(room.attrib['Name'], output.attrib['IntegrationID'], output.attrib['Name'])
                    props = {
                        PROP_ROOM : room.attrib['Name'], 
                        PROP_GATEWAY: gatewayID,
                        PROP_INTEGRATION_ID : output.attrib['IntegrationID'],
                        PROP_OUTPUTTYPE: output.attrib[PROP_OUTPUTTYPE]
                    }
                    self.createLutronDevice(DEV_SHADE, name, address, props, room.attrib['Name'])

                elif output.attrib['OutputType'] == "CEILING_FAN_TYPE":
                    address = gatewayID + ":" + output.attrib['IntegrationID']
                    name = u"{} - Fan {} - {}".format(room.attrib['Name'], output.attrib['IntegrationID'], output.attrib['Name'])
                    props = {
                        PROP_ROOM : room.attrib['Name'], 
                        PROP_GATEWAY: gatewayID,
                        PROP_INTEGRATION_ID : output.attrib['IntegrationID'],
                        PROP_OUTPUTTYPE: output.attrib[PROP_OUTPUTTYPE]
                    }
                    self.createLutronDevice(DEV_FAN, name, address, props, room.attrib['Name'])

                elif output.attrib['OutputType'] == "CCO_PULSED":
                    address = gatewayID + ":" + output.attrib['IntegrationID']
                    name = u"{} - VCRX CCO Momentary {} - {}".format(room.attrib['Name'], output.attrib['IntegrationID'], output.attrib['Name'])
                    props = {
                        PROP_ROOM : room.attrib['Name'], 
                        PROP_GATEWAY: gatewayID,
                        PROP_INTEGRATION_ID : output.attrib['IntegrationID'], 
                        PROP_CCO_TYPE : "momentary", 
                        PROP_SUPPORTS_STATUS_REQUEST : "False",
                        PROP_OUTPUTTYPE: output.attrib[PROP_OUTPUTTYPE]
                    }
                    self.createLutronDevice(DEV_CCO, name, address, props, room.attrib['Name'])

                elif output.attrib['OutputType'] == "CCO_MAINTAINED":
                    address = gatewayID + ":" + output.attrib['IntegrationID']
                    name = u"{} - VCRX CCO Sustained {} - {}".format(room.attrib['Name'], output.attrib['IntegrationID'], output.attrib['Name'])
                    props = {
                        PROP_ROOM : room.attrib['Name'], 
                        PROP_GATEWAY: gatewayID,
                        PROP_INTEGRATION_ID : output.attrib['IntegrationID'], 
                        PROP_CCO_TYPE : "sustained", 
                        PROP_SUPPORTS_STATUS_REQUEST : "True",
                        PROP_OUTPUTTYPE: output.attrib[PROP_OUTPUTTYPE]
                    }
                    self.createLutronDevice(DEV_CCO, name, address, props, room.attrib['Name'])

                elif output.attrib['OutputType'] == "HVAC":
                    pass

                else:
                    self.logger.error("Unknown Output Type: {} ({}, {})".format(output.attrib['Name'], output.attrib['OutputType'], output.attrib['IntegrationID']))


            for device in room.findall('DeviceGroups/DeviceGroup/Devices/Device'):
                self.logger.debug("Device: %s (%s,%s)" % (device.attrib['Name'], device.attrib['IntegrationID'], device.attrib['DeviceType']))

                if device.attrib['DeviceType'] == "SEETOUCH_KEYPAD" or device.attrib['DeviceType'] == "HYBRID_SEETOUCH_KEYPAD" or device.attrib['DeviceType'] == "SEETOUCH_TABLETOP_KEYPAD":  
                    for component in device.findall('Components/Component'):
                        self.logger.debug("Component: %s (%s)" % (component.attrib['ComponentNumber'], component.attrib['ComponentType']))
                        if component.attrib['ComponentType'] == "BUTTON":

                            assignments = len(component.findall('Button/Actions/Action/Presets/Preset/PresetAssignments/PresetAssignment'))  
                            if not self.create_unused_keypad and assignments == 0:
                                continue
                            
                            address = gatewayID + ":" + device.attrib['IntegrationID'] + "." + component.attrib['ComponentNumber']
                            keypadType = device.attrib['DeviceType']
                            buttonNum = int(component.attrib['ComponentNumber'])
                            if ((keypadType == "SEETOUCH_KEYPAD") or (keypadType == "HYBRID_SEETOUCH_KEYPAD")) and (buttonNum == 16):
                                name = u"{} - {} - Button {:03}.{:02} - Top Lower".format(room.attrib['Name'], device.attrib['Name'], int(device.attrib['IntegrationID']), int(component.attrib['ComponentNumber']))
                            elif ((keypadType == "SEETOUCH_KEYPAD") or (keypadType == "HYBRID_SEETOUCH_KEYPAD")) and (buttonNum == 17):
                                name = u"{} - {} - Button {:03}.{:02} - Top Raise".format(room.attrib['Name'], device.attrib['Name'], int(device.attrib['IntegrationID']), int(component.attrib['ComponentNumber']))
                            elif ((keypadType == "SEETOUCH_KEYPAD") or (keypadType == "HYBRID_SEETOUCH_KEYPAD")) and (buttonNum == 18):
                                name = u"{} - {} - Button {:03}.{:02} - Bottom Lower".format(room.attrib['Name'], device.attrib['Name'], int(device.attrib['IntegrationID']), int(component.attrib['ComponentNumber']))
                            elif ((keypadType == "SEETOUCH_KEYPAD") or (keypadType == "HYBRID_SEETOUCH_KEYPAD")) and (buttonNum == 19):
                                name = u"{} - {} - Button {:03}.{:02} - Bottom Raise".format(room.attrib['Name'], device.attrib['Name'], int(device.attrib['IntegrationID']), int(component.attrib['ComponentNumber']))
                            elif (keypadType == "SEETOUCH_TABLETOP_KEYPAD") and (buttonNum == 20):
                                name = u"{} - {} - Button {:03}.{:02}} - Column 1 Lower".format(room.attrib['Name'], device.attrib['Name'], int(device.attrib['IntegrationID']), int(component.attrib['ComponentNumber']))
                            elif (keypadType == "SEETOUCH_TABLETOP_KEYPAD") and (buttonNum == 21):
                                name = u"{} - {} - Button {:03}.{:02} - Column 1 Raise".format(room.attrib['Name'], device.attrib['Name'], int(device.attrib['IntegrationID']), int(component.attrib['ComponentNumber']))
                            elif (keypadType == "SEETOUCH_TABLETOP_KEYPAD") and (buttonNum == 22):
                                name = u"{} - {} - Button {:03}.{:02}} - Column 2 Lower".format(room.attrib['Name'], device.attrib['Name'], int(device.attrib['IntegrationID']), int(component.attrib['ComponentNumber']))
                            elif (keypadType == "SEETOUCH_TABLETOP_KEYPAD") and (buttonNum == 23):
                                name = u"{} - {} - Button {:03}.{:02} - Column 2 Raise".format(room.attrib['Name'], device.attrib['Name'], int(device.attrib['IntegrationID']), int(component.attrib['ComponentNumber']))
                            elif (keypadType == "SEETOUCH_TABLETOP_KEYPAD") and (buttonNum == 24):
                                name = u"{} - {} - Button {:03}.{:02} - Column 3 Lower".format(room.attrib['Name'], device.attrib['Name'], int(device.attrib['IntegrationID']), int(component.attrib['ComponentNumber']))
                            elif (keypadType == "SEETOUCH_TABLETOP_KEYPAD") and (buttonNum == 25):
                                name = u"{} - {} - Button {:03}.{:02} - Column 3 Raise".format(room.attrib['Name'], device.attrib['Name'], int(device.attrib['IntegrationID']), int(component.attrib['ComponentNumber']))
                            else:
                                name = u"{} - {} - Button {:03}.{:02}".format(room.attrib['Name'], device.attrib['Name'], int(device.attrib['IntegrationID']), int(component.attrib['ComponentNumber']))
                                try:                          
                                    engraving = component.find("Button").attrib['Engraving']
                                    name = name + " - " + engraving
                                except:
                                    pass

                            try:
                                buttonType = component.find("Button").attrib[PROP_BUTTONTYPE]
                            except:
                                buttonType = u"Unknown"
                            props = {
                                PROP_ROOM : room.attrib['Name'], 
                                PROP_LIST_TYPE : "button", 
                                PROP_GATEWAY: gatewayID,
                                PROP_INTEGRATION_ID : device.attrib['IntegrationID'], 
                                PROP_COMPONENT_ID : component.attrib['ComponentNumber'], 
                                PROP_KEYPADBUT_DISPLAY_LED_STATE : "false", 
                                PROP_BUTTONTYPE : buttonType,
                               PROP_ISBUTTON : "True"
                            }
                            self.createLutronDevice(DEV_KEYPAD, name, address, props, room.attrib['Name'])
                    
                            # create button LED, if needed for the button
                            
                            if ((keypadType == "SEETOUCH_KEYPAD") or (keypadType == "HYBRID_SEETOUCH_KEYPAD")) and (buttonNum > 6):
                                continue
                            if (keypadType == "SEETOUCH_TABLETOP_KEYPAD") and (buttonNum > 17):
                                continue
                                            
                            name = name + " LED"  
                            keypadLED = str(int(component.attrib['ComponentNumber']) + 80)
                            address = gatewayID + ":" + device.attrib['IntegrationID'] + "." + keypadLED
                            props = {
                                PROP_ROOM : room.attrib['Name'], 
                                PROP_LIST_TYPE : "LED", 
                                PROP_GATEWAY: gatewayID,
                                PROP_INTEGRATION_ID : device.attrib['IntegrationID'], 
                                PROP_COMPONENT_ID : keypadLED, 
                                PROP_KEYPADBUT_DISPLAY_LED_STATE : "False" 
                            }
                            self.createLutronDevice(DEV_KEYPAD, name, address, props, room.attrib['Name'])

                        elif component.attrib['ComponentType'] == "LED":
                            pass    # LED device created same time as button
                    
                        else:
                            self.logger.error("Unknown Component Type: %s (%s)" % (component.attrib['Name'], component.attrib['ComponentType']))
                                         
                elif device.attrib['DeviceType'] == "VISOR_CONTROL_RECEIVER":
                    for component in device.findall('Components/Component'):
                        self.logger.debug("Component: %s (%s)" % (component.attrib['ComponentNumber'], component.attrib['ComponentType']))
                        if component.attrib['ComponentType'] == "BUTTON":
                            assignments = len(component.findall('Button/Actions/Action/Presets/Preset/PresetAssignments/PresetAssignment'))                            
                            if not self.create_unused_keypad and assignments == 0:
                                continue

                            name = u"{} - VCRX Button {:03}.{:02}".format(room.attrib['Name'], int(device.attrib['IntegrationID']), int(component.attrib['ComponentNumber']))
                            try:                          
                                engraving = component.find("Button").attrib['Engraving']
                                name = name + " - " + engraving
                            except:
                                pass
                            address = gatewayID + ":" + device.attrib['IntegrationID'] + "." + component.attrib['ComponentNumber']

                            try:
                                buttonType = component.find("Button").attrib[PROP_BUTTONTYPE]
                            except:
                                buttonType = u"Unknown"
                            props = {
                                PROP_ROOM : room.attrib['Name'], 
                                PROP_LIST_TYPE : "button", 
                                PROP_GATEWAY: gatewayID,
                                PROP_INTEGRATION_ID : device.attrib['IntegrationID'], 
                                PROP_COMPONENT_ID : component.attrib['ComponentNumber'], 
                                PROP_KEYPADBUT_DISPLAY_LED_STATE : "false", 
                                PROP_BUTTONTYPE : buttonType,
                                PROP_ISBUTTON : "True"
                            }
                            self.createLutronDevice(DEV_KEYPAD, name, address, props, room.attrib['Name'])
                                
                            # create button LED, if needed for the button

                            name = name + " LED"  
                            keypadLED = str(int(component.attrib['ComponentNumber']) + 80)
                            address = gatewayID + ":" + device.attrib['IntegrationID'] + "." + keypadLED
                            props = {
                                PROP_ROOM : room.attrib['Name'], 
                                PROP_LIST_TYPE : "LED", 
                                PROP_GATEWAY: gatewayID,
                                PROP_INTEGRATION_ID : device.attrib['IntegrationID'], 
                                PROP_COMPONENT_ID : keypadLED, 
                                PROP_KEYPADBUT_DISPLAY_LED_STATE : "False" 
                            }
                            self.createLutronDevice(DEV_KEYPAD, name, address, props, room.attrib['Name'])

                        elif component.attrib['ComponentType'] == "LED":
                            pass
                            
                        elif component.attrib['ComponentType'] == "CCI":
                            name = u"{} - VCRX CCI Input {:03}.{:02}".format(room.attrib['Name'], int(device.attrib['IntegrationID']), int(component.attrib['ComponentNumber']))
                            address = gatewayID + ":" + device.attrib['IntegrationID'] + "." + component.attrib['ComponentNumber']
                            props = {
                                PROP_ROOM : room.attrib['Name'], 
                                PROP_GATEWAY: gatewayID,
                                PROP_INTEGRATION_ID : device.attrib['IntegrationID'], 
                                PROP_COMPONENT_ID : component.attrib['ComponentNumber'], 
                                PROP_SUPPORTS_STATUS_REQUEST : "False" 
                            }
                            self.createLutronDevice(DEV_CCI, name, address, props, room.attrib['Name'])

                        else:
                            self.logger.error("Unknown Component Type: %s (%s)" % (component.attrib['Name'], component.attrib['ComponentType']))
                                         
                elif device.attrib['DeviceType'] == "PICO_KEYPAD":
                    for component in device.findall('Components/Component'):
                        self.logger.debug("Component: %s (%s)" % (component.attrib['ComponentNumber'], component.attrib['ComponentType']))
                        if component.attrib['ComponentType'] == "BUTTON":

                            assignments = len(component.findall('Button/Actions/Action/Presets/Preset/PresetAssignments/PresetAssignment'))                            
                            if not self.create_unused_keypad and assignments == 0:
                                continue

                            name = u"{} - {} - Button {:03}.{:02}".format(room.attrib['Name'], device.attrib['Name'], int(device.attrib['IntegrationID']), int(component.attrib['ComponentNumber']))
                            try:                          
                                engraving = component.find("Button").attrib['Engraving']
                                name = name + " - " + engraving
                            except:
                                pass
                            address = gatewayID + ":" + device.attrib['IntegrationID'] + "." + component.attrib['ComponentNumber']

                            try:
                                buttonType = component.find("Button").attrib[PROP_BUTTONTYPE]
                            except:
                                buttonType = u"Unknown"
                            props = {
                                PROP_ROOM : room.attrib['Name'], 
                                PROP_GATEWAY: gatewayID,
                                PROP_INTEGRATION_ID : device.attrib['IntegrationID'], 
                                PROP_COMPONENT_ID : component.attrib['ComponentNumber'], 
                                PROP_BUTTONTYPE : buttonType,
                                PROP_ISBUTTON : "True"
                            }
                            self.createLutronDevice(DEV_PICO, name, address, props, room.attrib['Name'])

                        else:
                            self.logger.error("Unknown Component Type: %s (%s)" % (component.attrib['Name'], component.attrib['ComponentType']))
                                         
                elif device.attrib['DeviceType'] == "MOTION_SENSOR":
                    name = u"{} - Motion Sensor {}".format(room.attrib['Name'], device.attrib['IntegrationID'])
                    address = gatewayID + ":" + device.attrib['IntegrationID']
                    props = {
                        PROP_ROOM : room.attrib['Name'], 
                        PROP_GATEWAY: gatewayID,
                        PROP_INTEGRATION_ID : address, 
                        PROP_SUPPORTS_STATUS_REQUEST : "False" 
                    }
                    self.createLutronDevice(DEV_SENSOR, name, address, props, room.attrib['Name'])
                    
                    # Create a Group (Room) device for every room that has a motion sensors
                    
                    name = u"Group {:03} - {}".format( int(room.attrib['IntegrationID']), room.attrib['Name'])
                    address = gatewayID + ":" + room.attrib['IntegrationID']
                    props = {
                        PROP_GATEWAY: gatewayID,
                        'group': address 
                    }
                    if not address in self.groups:
                        self.createLutronDevice(DEV_GROUP, name, address, props, room.attrib['Name'])
                   
                    if self.create_group_triggers:
                        self.logger.debug("Creating Group triggers for: {} ({})".format(name, address))

                        if "Lutron" in indigo.triggers.folders:
                            theFolder = indigo.triggers.folders["Lutron"].id
                        else:
                            self.logger.debug("Creating Trigger Folder: '%s'" % ("Lutron"))            
                            theFolder = indigo.triggers.folder.create("Lutron").id
                                                    
                        trigger_exists = False
                        for triggerId, trigger in self.groupTriggers.iteritems():
                            if (trigger.pluginProps[PROP_GROUP] == address) and (trigger.pluginProps["occupancyPopUp"] == "3"):
                                trigger_exists = True
                                break

                        if trigger_exists:
                            self.logger.debug("Skipping existing group trigger: {}, {}".format(trigger.pluginProps[PROP_GROUP], trigger.pluginProps["occupancyPopUp"]))
                
                        else:
                            triggerName = u"{} Occupied".format(name)
                            self.logger.info("Creating groupEvent trigger: '%s' (%s)" % (triggerName, address))
                            indigo.pluginEvent.create(name=triggerName, 
                                description="", 
                                folder=theFolder,
                                pluginId="com.jimandnoreen.indigoplugin.lutron-radiora2",
                                pluginTypeId="groupEvent",
                                props={PROP_GROUP : address, "occupancyPopUp": "3"}
                            )
                                             
                        trigger_exists = False
                        for triggerId, trigger in self.groupTriggers.iteritems():
                            if (trigger.pluginProps[PROP_GROUP] == address) and (trigger.pluginProps["occupancyPopUp"] == "4"):
                                trigger_exists = True
                                break

                        if trigger_exists:
                            self.logger.debug("Skipping existing group trigger: {}, {}".format(trigger.pluginProps[PROP_GROUP], trigger.pluginProps["occupancyPopUp"]))
                
                        else:
                            triggerName = u"{} Unoccupied".format(name)
                            self.logger.info("Creating groupEvent trigger: '%s' (%s)" % (triggerName, address))
                            indigo.pluginEvent.create(name=triggerName, 
                                description="", 
                                folder=theFolder,
                                pluginId="com.jimandnoreen.indigoplugin.lutron-radiora2",
                                pluginTypeId="groupEvent",
                                props={PROP_GROUP : address, "occupancyPopUp": "4"}
                            )
 
                elif device.attrib['DeviceType'] == "TEMPERATURE_SENSOR":
                    pass
                                                             
                else:
                    self.logger.error("Unknown Device Type: %s (%s)" % (device.attrib['Name'], device.attrib['DeviceType']))
                    
        self.logger.info("Finding Timeclock events...")
        for event in root.iter('TimeClockEvent'):
            self.logger.debug("TimeClockEvent: %s (%s)" % (event.attrib['Name'], event.attrib['EventNumber']))
            name = u"Event {:02} - {}".format(int(event.attrib['EventNumber']), event.attrib['Name'])
            address = gatewayID + ":" + "Event.{}".format(event.attrib['EventNumber'])
            props = {
                PROP_GATEWAY: gatewayID,
                PROP_EVENT : event.attrib['EventNumber']
            }
            self.createLutronDevice(DEV_TIMECLOCKEVENT, name, address, props, "TimeClock")
            
            if self.create_event_triggers:
                self.logger.debug("Creating Event triggers for: {} ({})".format(name, address))

                if "Lutron" in indigo.triggers.folders:
                    theFolder = indigo.triggers.folders["Lutron"].id
                else:
                    self.logger.debug("Creating Trigger Folder: '%s'" % ("Lutron"))            
                    theFolder = indigo.triggers.folder.create("Lutron").id
        
                trigger_exists = False
                for triggerId, trigger in self.eventTriggers.iteritems():
                    if trigger.pluginProps[PROP_EVENT] == event.attrib['EventNumber']:
                        trigger_exists = True
                        break

                if trigger_exists:
                    self.logger.debug("Skipping existing event trigger: {}".format(trigger.pluginProps[PROP_EVENT]))
                
                else:
                    triggerName = u"{} Trigger".format(name)
                    self.logger.info("Creating timeClockEvent trigger: '{}' ({})".format(triggerName, event.attrib['EventNumber']))
                    indigo.pluginEvent.create(name=triggerName, 
                        description="", 
                        folder=theFolder,
                        pluginId="com.jimandnoreen.indigoplugin.lutron-radiora2",
                        pluginTypeId="timeClockEvent",
                        props={PROP_EVENT : event.attrib['EventNumber']}
                    )
            
        self.logger.info("Finding HVAC devices...")
        for hvac in root.iter('HVAC'):
            self.logger.debug("HVAC: %s (%s)" % (hvac.attrib['Name'], hvac.attrib['IntegrationID']))
            name = u"HVAC {:03} - {}".format(int(hvac.attrib['IntegrationID']), hvac.attrib['Name'])
            address = gatewayID + ":" + hvac.attrib['IntegrationID']
            props = {
                PROP_GATEWAY: gatewayID,
                PROP_INTEGRATION_ID: hvac.attrib['IntegrationID']
            }
            self.createLutronDevice(DEV_THERMO, name, address, props, "HVAC")
                     
                        
        self.logger.info(u"Creating Devices done.")        
        self.threadLock.release()
        return


    def createLutronDevice(self, devType, name, address, props, room):
        
        self.logger.threaddebug("createLutronDevice: devType = {}, name = {}, address = {}, props = {}, room = {}".format(devType, name, address, props, room))            

        folderNameDict = {
            DEV_PHANTOM_BUTTON   : "Lutron Phantom Buttons",
            DEV_DIMMER           : "Lutron Dimmers",
            DEV_SWITCH           : "Lutron Switches",
            DEV_KEYPAD           : "Lutron Keypads",
            DEV_FAN              : "Lutron Fans",
            DEV_SENSOR           : "Lutron Sensors",
            DEV_THERMO           : "Lutron Thermostats",
            DEV_CCO              : "Lutron Switches",
            DEV_CCI              : "Lutron Sensors",
            DEV_SHADE            : "Lutron Shades",
            DEV_PICO             : "Lutron Keypads",
            DEV_GROUP            : "Lutron Room Groups",
            DEV_TIMECLOCKEVENT   : "Lutron Timeclock Events"
        }

        # first, make sure this device doesn't exist.  Unless I screwed up, the addresses should be unique
        # it would be more efficient to search through the internal device lists, but a pain to code.
        # If it does exist, update with the new properties
        
        for dev in indigo.devices.iter("self"):        
            if dev.address == address:
                if dev.pluginProps.get(PROP_ROOM, None):
                    self.logger.debug("Skipping existing device: '{}' ({})".format(name, address))            
                    return dev
                else:
                    self.logger.debug("Adding ROOM property to existing device: '{}' ({})".format(name, address))            
                    self.update_device_property(dev, PROP_ROOM, new_value = room)
                    return dev
            
        # Pick the folder for this device, create it if necessary
        
        if self.group_by == "Type":                     
            folderName = folderNameDict[devType]
            if folderName in indigo.devices.folders:
                theFolder = indigo.devices.folders[folderName].id
            else:
                self.logger.debug("Creating Device Folder: '%s'" % (folderName))            
                theFolder = indigo.devices.folder.create(folderName).id
                
        elif self.group_by == "Room":
            folderName = u"Lutron {}".format(room)
            if folderName in indigo.devices.folders:
                theFolder = indigo.devices.folders[folderName].id
            else:
                self.logger.debug("Creating Device Folder: '{}'".format(folderName))            
                theFolder = indigo.devices.folder.create(folderName).id
                
        elif self.group_by == "None":
            folderName = u"DEVICES"
            theFolder = 0

        else:
            self.logger.error("Unknown value for group_by")
            return
                                        
        # finally, create the device
        
        self.logger.info("Creating %s device: '%s' (%s) in '%s'" % (devType, name, address, folderName))
        try:
            newDevice = indigo.device.create(indigo.kProtocol.Plugin, address=address, name=name, deviceTypeId=devType, props=props, folder=theFolder)
        except Exception, e:
            self.logger.error("Error calling indigo.device.create(): %s" % (e.message))
            newDevice = None
                                                    
        return newDevice
        

    #################################
    #
    #  Future versions: implement additional thermostat actions, shades (define as dimmers for now)

