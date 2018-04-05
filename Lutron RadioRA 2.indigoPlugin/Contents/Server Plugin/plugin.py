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
RA_LINKEDDEVICE = "ra2LinkedDevice"
RA_TIMECLOCKEVENT = "ra2TimeClockEvent"
RA_GROUP = "ra2Group"

# pluginProps keys
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

        self.queryAtStartup = self.pluginPrefs.get(u"queryAtStartup", False)

        self.connSerial = {}
        self.command = ''
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
        self.runstartup = False
        self.IP = False     # Default to serial I/O, not IP -vic13
        self.portEnabled = False
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

        try:
            self.IP = self.pluginPrefs["IP"]
        except KeyError:
            self.logger.warning(u"Plugin not yet configured.\nPlease save the configuration then reload the plugin.\nThis should only happen the first time you run the plugin\nor if you delete the preferences file.")
            return

        savedList = indigo.activePlugin.pluginPrefs.get(u"linkedDevices", None)
        if savedList:
            self.linkedDeviceList = json.loads(savedList)
        else:
            self.linkedDeviceList = {}        
        self.logLinkedDevices()


        if self.IP:
            self.ipStartup()
        else:
            self.serialStartup()
        self.runstartup = False

        indigo.devices.subscribeToChanges()

        if self.queryAtStartup:
            self.queryAllDevices()


    def shutdown(self):
        self.logger.info(u"Shutting down Lutron")
        if self.IP:
            self.connIP.close()
  
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
        return False      

              
    def deviceStartComm(self, dev):
        if dev.deviceTypeId == RA_PHANTOM_BUTTON:
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

            address = u"{}.{}".format(dev.pluginProps[PROP_INTEGRATION_ID], dev.pluginProps[PROP_COMPONENT_ID])
            self.phantomButtons[address] = dev
            self.update_device_property(dev, "address", address)
            
        elif dev.deviceTypeId == RA_DIMMER:
            if dev.pluginProps.get(PROP_INTEGRATION_ID, None) == None:
                self.update_device_property(dev, PROP_INTEGRATION_ID, dev.pluginProps[PROP_ZONE])
                self.remove_device_property(dev, PROP_ZONE)
                self.logger.info(u"{}: Updated zone property to IntegrationID".format(dev.name))

            address = u"{}".format(dev.pluginProps[PROP_INTEGRATION_ID])
            self.dimmers[address] = dev
            self.update_device_property(dev, "address", address)
            
        elif dev.deviceTypeId == RA_SHADE:
            if dev.pluginProps.get(PROP_INTEGRATION_ID, None) == None:
                self.update_device_property(dev, PROP_INTEGRATION_ID, dev.pluginProps[PROP_SHADE])
                self.remove_device_property(dev, PROP_SHADE)
                self.logger.info(u"{}: Updated zone property to IntegrationID".format(dev.name))

            address = u"{}".format(dev.pluginProps[PROP_INTEGRATION_ID])
            self.shades[address] = dev
            self.update_device_property(dev, "address", address)
            dev.updateStateImageOnServer( indigo.kStateImageSel.None)
            
        elif dev.deviceTypeId == RA_SWITCH:
            if dev.pluginProps.get(PROP_INTEGRATION_ID, None) == None:
                self.update_device_property(dev, PROP_INTEGRATION_ID, dev.pluginProps[PROP_SWITCH])
                self.remove_device_property(dev, PROP_SWITCH)
                self.logger.info(u"{}: Updated switch property to IntegrationID".format(dev.name))

            address = u"{}".format(dev.pluginProps[PROP_INTEGRATION_ID])
            self.switches[address] = dev
            self.update_device_property(dev, "address", address)
            
        elif dev.deviceTypeId == RA_FAN:
            if dev.pluginProps.get(PROP_INTEGRATION_ID, None) == None:
                self.update_device_property(dev, PROP_INTEGRATION_ID, dev.pluginProps[PROP_FAN])
                self.remove_device_property(dev, PROP_FAN)
                self.logger.info(u"{}: Updated fan property to IntegrationID".format(dev.name))

            address = u"{}".format(dev.pluginProps[PROP_INTEGRATION_ID])
            self.fans[address] = dev
            self.update_device_property(dev, "address", address)

        elif dev.deviceTypeId == RA_THERMO:
            if dev.pluginProps.get(PROP_INTEGRATION_ID, None) == None:
                self.update_device_property(dev, PROP_INTEGRATION_ID, dev.pluginProps[PROP_THERMO])
                self.remove_device_property(dev, PROP_THERMO)
                self.logger.info(u"{}: Updated thermo property to IntegrationID".format(dev.name))

            address = u"{}".format(dev.pluginProps[PROP_INTEGRATION_ID])
            self.thermos[address] = dev
            self.update_device_property(dev, "address", address)
            
        elif dev.deviceTypeId == RA_KEYPAD:
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
                
            address = u"{}.{}".format(dev.pluginProps[PROP_INTEGRATION_ID], dev.pluginProps[PROP_COMPONENT_ID])
            self.update_device_property(dev, "address", address)
            if int(dev.pluginProps[PROP_COMPONENT_ID]) > 80:
                self.update_device_property(dev, PROP_KEYPADBUT_DISPLAY_LED_STATE, new_value = dev.pluginProps[PROP_KEYPADBUT_DISPLAY_LED_STATE])
            else:
                self.update_device_property(dev, PROP_KEYPADBUT_DISPLAY_LED_STATE, new_value = False)
            self.keypads[address] = dev

        elif dev.deviceTypeId == RA_SENSOR:
            if dev.pluginProps.get(PROP_INTEGRATION_ID, None) == None:
                self.update_device_property(dev, PROP_INTEGRATION_ID, dev.pluginProps[PROP_SENSOR])
                self.remove_device_property(dev, PROP_SENSOR)
                self.logger.info(u"{}: Updated sensor property to IntegrationID".format(dev.name))

            address = u"{}".format(dev.pluginProps[PROP_INTEGRATION_ID])
            self.sensors[address] = dev
            self.update_device_property(dev, "address", address)

        elif dev.deviceTypeId == RA_CCI:
            if dev.pluginProps.get(PROP_INTEGRATION_ID, None) == None:
                self.update_device_property(dev, PROP_INTEGRATION_ID, dev.pluginProps[PROP_CCI_INTEGRATION_ID])
                self.remove_device_property(dev, PROP_CCI_INTEGRATION_ID)
                self.logger.info(u"{}: Updated cciIntegrationID property to IntegrationID".format(dev.name))

            if dev.pluginProps.get(PROP_COMPONENT, None):
                self.update_device_property(dev, PROP_COMPONENT_ID, dev.pluginProps[PROP_COMPONENT])
                self.remove_device_property(dev, PROP_COMPONENT)
                self.logger.info(u"{}: Updated cciCompoment property to componentID".format(dev.name))
                
            address = u"{}.{}".format(dev.pluginProps[PROP_INTEGRATION_ID], dev.pluginProps[PROP_COMPONENT_ID])
            self.ccis[address] = dev
            self.update_device_property(dev, "address", address)

        elif dev.deviceTypeId == RA_CCO:
            if dev.pluginProps.get(PROP_INTEGRATION_ID, None) == None:
                self.update_device_property(dev, PROP_INTEGRATION_ID, dev.pluginProps[PROP_CCO_INTEGRATION_ID])
                self.remove_device_property(dev, PROP_CCO_INTEGRATION_ID)
                self.logger.info(u"{}: Updated ccoIntegrationID property to IntegrationID".format(dev.name))

            address = u"{}".format(dev.pluginProps[PROP_INTEGRATION_ID])
            self.ccos[address] = dev
            self.update_device_property(dev, "address", address)

            ccoType = dev.pluginProps[PROP_CCO_TYPE]
            if ccoType == "momentary":
                dev.updateStateOnServer("onOffState", False)
            else:
                self.update_device_property(dev, PROP_SUPPORTS_STATUS_REQUEST, new_value = True)
            
        elif dev.deviceTypeId == RA_PICO:
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
                
            address = u"{}.{}".format(dev.pluginProps[PROP_INTEGRATION_ID], dev.pluginProps[PROP_COMPONENT_ID])
            self.picos[address] = dev
            self.update_device_property(dev, "address", address)

        elif dev.deviceTypeId == RA_TIMECLOCKEVENT:
            address = u"Event.{}".format(dev.pluginProps[PROP_EVENT])
            self.events[address] = dev
            self.update_device_property(dev, "address", address)

        elif dev.deviceTypeId == RA_GROUP:
            address = u"{}".format(dev.pluginProps[PROP_GROUP])
            self.groups[address] = dev
            self.update_device_property(dev, "address", address)

        elif dev.deviceTypeId == RA_LINKEDDEVICE:
        
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
            self.logger.error(u"{}: deviceStopComm: Unknown device type: {}".format(dev.name, dev.deviceTypeId))
            return
        
        if (dev.pluginProps.get(PROP_ISBUTTON, None)):      # it's a button, so put it in the tree
            try:
                roomName = dev.pluginProps[PROP_ROOM]
            except:
                roomname = u"Unknown"
            try:
                room = self.roomButtonTree[roomName]
            except:
                room = {}
                self.roomButtonTree[roomName] = room
            self.roomButtonTree[roomName][dev.id] = dev.name
        
        
    def deviceStopComm(self, dev):
        if dev.deviceTypeId == RA_PHANTOM_BUTTON:
            del self.phantomButtons[dev.address]

        elif dev.deviceTypeId == RA_DIMMER:
            del self.dimmers[dev.address]

        elif dev.deviceTypeId == RA_SHADE:
            del self.shades[dev.address]

        elif dev.deviceTypeId == RA_SWITCH:
            del self.switches[dev.address]

        elif dev.deviceTypeId == RA_KEYPAD:
            del self.keypads[dev.address]

        elif dev.deviceTypeId == RA_FAN:
            del self.fans[dev.address]

        elif dev.deviceTypeId == RA_THERMO:
            del self.thermos[dev.address]

        elif dev.deviceTypeId == RA_SENSOR:
            del self.sensors[dev.address]

        elif dev.deviceTypeId == RA_CCI:
            del self.ccis[dev.address]

        elif dev.deviceTypeId == RA_CCO:
            del self.ccos[dev.address]

        elif dev.deviceTypeId == RA_PICO:
            del self.picos[dev.address]

        elif dev.deviceTypeId == RA_GROUP:
            del self.groups[dev.address]

        elif dev.deviceTypeId == RA_TIMECLOCKEVENT:
            del self.events[dev.address]

        elif dev.deviceTypeId == RA_LINKEDDEVICE:
            pass

        else:
            self.logger.error(u"{}: deviceStopComm: Unknown device type: {}".format(dev.name, dev.deviceTypeId))

    ########################################

    def validateDeviceConfigUi(self, valuesDict, typeId, devId):
        self.logger.debug(u"validateDeviceConfigUi: typeId = {}, devId = {}".format(typeId, devId))

        errorsDict = indigo.Dict()

        if typeId == RA_KEYPAD and bool(valuesDict[PROP_KEYPADBUT_DISPLAY_LED_STATE]) and int(valuesDict[PROP_KEYPADBUT]) < 80:
            valuesDict[PROP_KEYPADBUT_DISPLAY_LED_STATE] = False
            self.logger.debug(u"validateDeviceConfigUi: forced PROP_KEYPADBUT_DISPLAY_LED_STATE to False for keypad # {}, button # {}".format(valuesDict[PROP_INTEGRATION_ID], valuesDict[PROP_KEYPADBUT]))
        
        if len(errorsDict) > 0:
            return (False, valuesDict, errorsDict)

        return (True, valuesDict)
        
    def runConcurrentThread(self):

        try:
            while True:

                if self.IP:
                    self.sleep(.1)
                    try:
                        if self.runstartup:
                            self.ipStartup()
                            self.runstartup = False

                        try:
                            self._processCommand(self.connIP.read_until("\n", self.timeout))
                        except:
                            pass
                            
                    except EOFError, e:
                        self.logger.error(u"EOFError: %s" % e.message)
                        if ('telnet connection closed' in e.message):
                            self.runstartup = True
                            self.sleep(10)
                    except AttributeError, e:
                        self.logger.debug(u"AttributeError: %s" % e.message)
                    except select.error, e:
                        self.logger.debug(u"Disconnected while listening: %s" % e.message)

                else:
                    while not self.portEnabled:
                        self.sleep(.1)

                    if self.runstartup:
                        self.serialStartup()
                        self.runstartup = False

                    s = self.connSerial.read()
                    if len(s) > 0:
                        # RadioRA 2 messages are always terminated with CRLF
                        if s == '\r':
                            self._processCommand(self.command)
                            self.command = ''
                        else:
                            self.command += s

        except self.StopThread:
            pass

    ####################

    def serialStartup(self):
        self.logger.info(u"Running serialStartup")

        self.portEnabled = False

        serialUrl = self.getSerialPortUrl(self.pluginPrefs, u"devicePort")
        self.logger.info(u"Serial Port URL is: " + serialUrl)

        self.connSerial = self.openSerial(u"Lutron RadioRA", serialUrl, 9600, stopbits=1, timeout=2, writeTimeout=1)
        if self.connSerial is None:
            self.logger.error(u"Failed to open serial port")
            return

        self.portEnabled = True

        # Disable main repeater terminal prompt
        self._sendCommand("#MONITORING,12,2")

        # Enable main repeater HVAC monitoring
        self._sendCommand("#MONITORING,17,1")

        # Enable main repeater monitoring param 18
        # (undocumented but seems to be enabled by default for ethernet connections)
        self._sendCommand("#MONITORING,18,1")


    def ipStartup(self):
        self.logger.info(u"Running ipStartup")
        self.timeout = 35   # Under some conditions Smart Bridge Pro takes a long time to connect

        host = self.pluginPrefs["ip_address"]

        try:
            self.logger.info(u"Connecting via IP to %s" % host)
            self.connIP = telnetlib.Telnet(host, 23, self.timeout)
        except socket.timeout:
            self.logger.error(u"Unable to connect to Lutron gateway. Timed out.")
            return
                    
        a = self.connIP.read_until(" ", self.timeout)
        self.logger.debug(u"self.connIP.read: %s" % a)

        if 'login' in a:
            self.logger.debug(u"Sending username.")
            self.connIP.write(str(self.pluginPrefs["ip_username"]) + "\r\n")

            a = self.connIP.read_until(" ", self.timeout)
            self.logger.debug(u"self.connIP.read: %s" % a)
            if 'password' in a:
                self.logger.debug(u"Sending password.")
                self.connIP.write(str(self.pluginPrefs["ip_password"]) + "\r\n")
            else:
                self.logger.debug(u"password failure.")
        else:
            self.logger.debug(u"username failure.")
        self.logger.debug(u"End of connection process.")
        self.timeout = 5   # Reset the timeout to something reasonable


#########################################
# Poll registered devices for status
#########################################
    def queryAllDevices(self):
        for dev in indigo.devices.iter("self"):
            indigo.device.statusRequest(dev)


# plugin configuration validation
    def validatePrefsConfigUi(self, valuesDict):
        errorDict = indigo.Dict()

        badAddr = "Please use either an IP address (i.e. 1.2.3.4) or a fully qualified host name (i.e. lutron.domain.com)"

        self.IP = valuesDict["IP"]

        if valuesDict["ip_address"].count('.') >= 3:
            ipOK = True
        else:
            ipOK = False

        try:
            if ipOK:
                rtn = True
            else:
                errorDict["ip_address"] = badAddr
                rtn = (False, valuesDict, errDict)
        except AttributeError:
            rtn = (True, valuesDict)

        try:
            if valuesDict["configDone"]:
                self.runstartup = False
            else:
                if ipOK and rtn:
                    self.logger.debug(u"Setting configDone to True")
                    valuesDict["configDone"] = True
                    self.logger.debug(u"Setting flag to run startup")
                    self.runstartup = True
        except KeyError:
            if ipOK and rtn:
                self.logger.debug(u"Setting configDone to True")
                valuesDict["configDone"] = True
                self.logger.exception(u"Setting flag to run startup")
                self.runstartup = True
        self.IP = valuesDict["IP"]
        self.logger.debug(u"%s, %s, %s" % (str(rtn), str(ipOK), str(self.IP)))
        return rtn

    def closedPrefsConfigUi(self, valuesDict, userCancelled):
        if not userCancelled:
            try:
                self.logLevel = int(valuesDict[u"logLevel"])
            except:
                self.logLevel = logging.INFO
            self.indigo_log_handler.setLevel(self.logLevel)
            self.logger.debug(u"logLevel = " + str(self.logLevel))

    ########################################

    def _processCommand(self, cmd):
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


    def _sendCommand(self, cmd):
        if self.IP:
            self.logger.debug(u"Sending network command:  %s" % cmd)
            cmd = cmd + "\r\n"
            try:
                self.connIP.write(str(cmd))
            except Exception, e:
                self.logger.warning(u"Error sending IP command, resetting connection:  %s", e)
                self.connIP.close()
                self.runstartup = True
        else:
            self.logger.debug(u"Sending serial command: %s" % cmd)
            cmd = cmd + "\r"
            self.connSerial.write(str(cmd))

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
                    zone.updateStateOnServer("onOffState", False)
                else:
                    zone.updateStateOnServer("onOffState", True)
                    zone.updateStateOnServer("brightnessLevel", int(level))
                self.logger.debug(u"Received: Dimmer " + zone.name + " level set to " + str(level))
                
            elif id in self.shades:
                shade = self.shades[id]
                if int(level) == 0:
                    shade.updateStateOnServer("onOffState", False)
                else:
                    shade.updateStateOnServer("onOffState", True)
                    shade.updateStateOnServer("brightnessLevel", int(level))
                self.logger.debug(u"Received: Shade " + shade.name + " opening set to " + str(level))
                
            elif id in self.switches:
                switch = self.switches[id]
                if int(level) == 0:
                    switch.updateStateOnServer("onOffState", False)
                    self.logger.debug(u"Received: Switch %s %s" % (switch.name, "turned Off"))
                else:
                    switch.updateStateOnServer("onOffState", True)
                    self.logger.debug(u"Received: Switch %s %s" % (switch.name, "turned On"))
                    
            elif id in self.ccos:
                cco = self.ccos[id]
                ccoType = cco.pluginProps[PROP_CCO_TYPE]
                if ccoType == "sustained":
                    if int(level) == 0:
                     cco.updateStateOnServer("onOffState", False)
                    else:
                     cco.updateStateOnServer("onOffState", True)
                if level == 0.0:
                    self.logger.debug(u"Received: CCO %s %s" % (cco.name, "Opened"))
                else:
                    self.logger.debug(u"Received: CCO %s %s" % (cco.name, "Closed"))
                    
            elif id in self.fans:
                fan = self.fans[id]
                if int(level) == 0:
                    fan.updateStateOnServer("onOffState", False)
                    fan.updateStateOnServer("speedIndex", 0)
                    fan.updateStateOnServer('actualSpeed', 0)
                elif level < 26.0:
                    fan.updateStateOnServer("onOffState", True)
                    fan.updateStateOnServer("speedIndex", 1)
                    fan.updateStateOnServer('actualSpeed', 25)
                elif level < 51.0:
                    fan.updateStateOnServer("onOffState", True)
                    fan.updateStateOnServer("speedIndex", 2)
                    fan.updateStateOnServer('actualSpeed', 50)
                elif level < 76.0:
                    fan.updateStateOnServer("onOffState", True)
                    fan.updateStateOnServer("speedIndex", 2)
                    fan.updateStateOnServer('actualSpeed', 75)
                else:
                    fan.updateStateOnServer("onOffState", True)
                    fan.updateStateOnServer("speedIndex", 3)
                    fan.updateStateOnServer('actualSpeed', 100)
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

        if self.IP:
            cmd = cmd.rstrip() # IP strings are terminated with \n -JL

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
                dev.updateStateOnServer("onOffState", False)
            elif status == '1':
                dev.updateStateOnServer("onOffState", True)

        if keypadid in self.keypads:
            self.logger.debug(u"Received a keypad button/LED status message: " + cmd)
            dev = self.keypads[keypadid]
            if status == '0':
                dev.updateStateOnServer("onOffState", False)
            elif status == '1':
                dev.updateStateOnServer("onOffState", True)
            
            if dev.pluginProps[PROP_KEYPADBUT_DISPLAY_LED_STATE]: # Also display this LED state on its corresponding button
            
                keypadid = id + '.' + str(int(button) - 80)         # Convert LED ID to button ID
                if keypadid in self.keypads:
                    keypad = self.keypads[keypadid]
                    self.logger.debug(u"Updating button status with state of LED ({}) for keypadID {}".format(status, keypadid))
                    if int(status) == 0:
                        keypad.updateStateOnServer("onOffState", False)
                    elif int(status) == 1:
                        keypad.updateStateOnServer("onOffState", True)
                else:
                    self.logger.error("WARNING: Invalid ID (%s) specified for LED.   Must be ID of button + 80.  Please correct and reload the plugin." % keypadid)
                    self.logger.debug(keypadid)

            if action == '3': # Check for triggers and linked devices
                self.buttonTriggerCheck(id, button)
                

        if keypadid in self.picos:
            self.logger.debug(u"Received a pico button status message: " + cmd)
            dev = self.picos[keypadid]
            if status == '0':
                dev.updateStateOnServer("onOffState", False)
            elif status == '1':
                dev.updateStateOnServer("onOffState", True)

            if action == '3': # Check for triggers and linked devices
                self.buttonTriggerCheck(id, button)

        if keypadid in self.ccis:
            self.logger.debug(u"Received a CCI status message: " + cmd)
            dev = self.ccis[keypadid]
            if status == '0':
                dev.updateStateOnServer("onOffState", False)
                self.logger.info(u"Received: CCI %s %s" % (cci.name, "Opened"))
            elif status == '1':
                dev.updateStateOnServer("onOffState", True)
                self.logger.info(u"Received: CCI %s %s" % (cci.name, "Closed"))

        if id in self.sensors:
            self.logger.debug(u"Received a sensor status message: " + cmd)
            dev = self.sensors[id]
            if status == '0':
                dev.updateStateOnServer("onOffState", False)
                self.logger.info(u"Received: Motion Sensor %s %s" % (but.name, "vacancy detected"))
            elif status == '1':
                dev.updateStateOnServer("onOffState", True)
                self.logger.info(u"Received: Motion Sensor %s %s" % (but.name, "motion detected"))

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
            if dev.deviceTypeId == RA_PHANTOM_BUTTON:
                integration_id = dev.pluginProps[PROP_INTEGRATION_ID]
                phantom_button = dev.pluginProps[PROP_COMPONENT_ID]
                sendCmd = ("#DEVICE," + str(int(integration_id)) + ","+ str(int(phantom_button)-100) + ",3,") # Press button
                
            elif dev.deviceTypeId == RA_PICO:
                pico = dev.pluginProps[PROP_INTEGRATION_ID]
                button = dev.pluginProps[PROP_COMPONENT_ID]
                sendCmd = ("#DEVICE," + pico + "," + button + ",3") # Press button
                
            elif dev.deviceTypeId == RA_KEYPAD:
                keypad = dev.pluginProps[PROP_INTEGRATION_ID]
                keypadButton = dev.pluginProps[PROP_COMPONENT_ID]
                if (int(keypadButton) > 80):
                    sendCmd = ("#DEVICE," + keypad + "," + str(int(keypadButton)) + ",9,1") # Turn on an LED
                else:
                    sendCmd = ("#DEVICE," + keypad + "," + str(int(keypadButton)) + ",3") # Press button
                    
            elif dev.deviceTypeId == RA_DIMMER:
                zone = dev.pluginProps[PROP_INTEGRATION_ID]
                sendCmd = ("#OUTPUT," + zone + ",1,100")
                self.lastBrightness[zone] = 100
                
            elif dev.deviceTypeId == RA_SHADE:
                shade = dev.pluginProps[PROP_INTEGRATION_ID]
                sendCmd = ("#OUTPUT," + shade + ",1,100")
                self.lastBrightness[shade] = 100
                
            elif dev.deviceTypeId == RA_SWITCH:
                switch = dev.pluginProps[PROP_INTEGRATION_ID]
                sendCmd = ("#OUTPUT," + switch + ",1,100")
                
            elif dev.deviceTypeId == RA_CCI:
                cci = dev.pluginProps[PROP_INTEGRATION_ID]
                component = dev.pluginProps[PROP_COMPONENT_ID]
                sendCmd = ("#DEVICE," + cci +"," + str(int(component)) + ",3")
                
            elif dev.deviceTypeId == RA_CCO:
                cco = dev.pluginProps[PROP_INTEGRATION_ID]
                ccoType = dev.pluginProps[PROP_CCO_TYPE]
                if ccoType == "momentary":
                    sendCmd = ("#OUTPUT," + cco + ",6")
                    sendCmd = ("#OUTPUT," + cco + ",1,1")
                else:
                    sendCmd = ("#OUTPUT," + cco + ",1,1")
            
        ###### TURN OFF ######
        elif action.deviceAction == indigo.kDeviceAction.TurnOff:
            if dev.deviceTypeId == RA_PHANTOM_BUTTON:
                phantom_button = dev.pluginProps[PROP_COMPONENT_ID]
                integration_id = dev.pluginProps[PROP_INTEGRATION_ID]
                sendCmd = ("#DEVICE," + str(int(integration_id)) + ","+ str(int(phantom_button)-100) + ",4,") # Release button
                
            elif dev.deviceTypeId == RA_PICO:
                pico = dev.pluginProps[PROP_INTEGRATION_ID]
                button = dev.pluginProps[PROP_COMPONENT_ID]
                sendCmd = ("#DEVICE," + pico + "," + button + ",4") # Release button
                
            elif dev.deviceTypeId == RA_KEYPAD:
                keypad = dev.pluginProps[PROP_INTEGRATION_ID]
                keypadButton = dev.pluginProps[PROP_COMPONENT_ID]
                if (int(keypadButton) > 80):
                    sendCmd = ("#DEVICE," + keypad + "," + str(int(keypadButton)) + ",9,0") # Turn off an LED
                else:
                    sendCmd = ("#DEVICE," + keypad + "," + str(int(keypadButton)) + ",4") # Release button
                    
            elif dev.deviceTypeId == RA_DIMMER:
                zone = dev.pluginProps[PROP_INTEGRATION_ID]
                sendCmd = ("#OUTPUT," + zone + ",1,0")
                self.lastBrightness[zone] = 0
                
            elif dev.deviceTypeId == RA_SHADE:
                shade = dev.pluginProps[PROP_INTEGRATION_ID]
                sendCmd = ("#OUTPUT," + shade + ",1,0")
                self.lastBrightness[shade] = 0
                
            elif dev.deviceTypeId == RA_SWITCH:
                switch = dev.pluginProps[PROP_INTEGRATION_ID]
                sendCmd = ("#OUTPUT," + switch + ",1,0")
                
            elif dev.deviceTypeId == RA_CCI:
                self.logger.debug(u"it is a cci")
                cci = dev.pluginProps[PROP_INTEGRATION_ID]
                component = dev.pluginProps[PROP_COMPONENT_ID]
                sendCmd = ("#DEVICE," + cci +"," + str(int(component)) + ",4")
                
            elif dev.deviceTypeId == RA_CCO:
                cco = dev.pluginProps[PROP_INTEGRATION_ID]
                ccoType = dev.pluginProps[PROP_CCO_TYPE]
                if ccoType == "momentary":
                    sendCmd = ("#OUTPUT," + cco + ",6")
                else:
                    sendCmd = ("#OUTPUT," + cco + ",1,0")

        ###### TOGGLE ######
        elif action.deviceAction == indigo.kDeviceAction.Toggle:
            if dev.deviceTypeId == RA_PHANTOM_BUTTON:
                integration_id = dev.pluginProps[PROP_INTEGRATION_ID]
                phantom_button = dev.pluginProps[PROP_COMPONENT_ID]
                sendCmd = ("#DEVICE," + str(int(integration_id)) + ","+ str(int(phantom_button)-100) + ",3,")
                
            elif dev.deviceTypeId == RA_KEYPAD:
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
                        
            elif dev.deviceTypeId == RA_DIMMER:
                zone = dev.pluginProps[PROP_INTEGRATION_ID]
                if dev.brightness > 0:
                    sendCmd = ("#OUTPUT," + zone + ",1,0")
                else:
                    sendCmd = ("#OUTPUT," + zone + ",1,100")
                    
            elif dev.deviceTypeId == RA_SHADE:
                shade = dev.pluginProps[PROP_INTEGRATION_ID]
                if dev.brightness > 0:
                    sendCmd = ("#OUTPUT," + shade + ",1,0")
                else:
                    sendCmd = ("#OUTPUT," + shade + ",1,100")
                    
            elif dev.deviceTypeId == RA_SWITCH:
                switch = dev.pluginProps[PROP_INTEGRATION_ID]
                if dev.onState == True:
                    sendCmd = ("#OUTPUT," + switch + ",1,0")
                else:
                    sendCmd = ("#OUTPUT," + switch + ",1,100")
                    
            elif dev.deviceTypeId == RA_CCI:
                self.logger.debug(u"it is a cci")
                cci = dev.pluginProps[PROP_INTEGRATION_ID]
                component = dev.pluginProps[PROP_COMPONENT_ID]
                if dev.onState == True:
                    sendCmd = ("#DEVICE," + cci +"," + str(int(component)) + ",4")
                else:
                    sendCmd = ("#DEVICE," + cci +"," + str(int(component)) + ",3")
                    
            elif dev.deviceTypeId == RA_CCO:
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
            if (dev.deviceTypeId == RA_DIMMER) or (dev.deviceTypeId == RA_SHADE):
                newBrightness = action.actionValue
                zone = dev.pluginProps[PROP_INTEGRATION_ID]
                sendCmd = ("#OUTPUT," + dev.pluginProps[PROP_INTEGRATION_ID] + ",1," + str(newBrightness))

        ###### BRIGHTEN BY ######
        elif action.deviceAction == indigo.kDimmerRelayAction.BrightenBy:
            if (dev.deviceTypeId == RA_DIMMER) or (dev.deviceTypeId == RA_SHADE):
                newBrightness = dev.brightness + action.actionValue
                if newBrightness > 100:
                    newBrightness = 100
                sendCmd = ("#OUTPUT," + dev.pluginProps[PROP_INTEGRATION_ID] + ",1," + str(newBrightness))

        ###### DIM BY ######
        elif action.deviceAction == indigo.kDimmerRelayAction.DimBy:
            if (dev.deviceTypeId == RA_DIMMER) or (dev.deviceTypeId == RA_SHADE):
                newBrightness = dev.brightness - action.actionValue
                if newBrightness < 0:
                    newBrightness = 0
                sendCmd = ("#OUTPUT," + dev.pluginProps[PROP_INTEGRATION_ID] + ",1," + str(newBrightness))

        ###### STATUS REQUEST ######
        elif action.deviceAction == indigo.kDeviceAction.RequestStatus:
            if dev.deviceTypeId == RA_PHANTOM_BUTTON:
                integration_id = dev.pluginProps[PROP_INTEGRATION_ID]
                phantom_button = dev.pluginProps[PROP_COMPONENT_ID]
                sendCmd = ("?DEVICE," + str(int(integration_id)) + ","+ str(int(phantom_button)) + ",9,")
                
            elif dev.deviceTypeId == RA_KEYPAD:
                integration_id = dev.pluginProps[PROP_INTEGRATION_ID]
                keypadButton = dev.pluginProps[PROP_COMPONENT_ID]
                if (int(keypadButton) > 80):
                    sendCmd = ("?DEVICE," + integration_id + "," + str(int(keypadButton)) + ",9")
                else:
                    sendCmd = ("?DEVICE," + integration_id + "," + str(int(keypadButton)+80) + ",9")
                    
            elif (dev.deviceTypeId == RA_DIMMER) or (dev.deviceTypeId == RA_SHADE) or (dev.deviceTypeId == RA_SWITCH):
                integration_id = dev.pluginProps[PROP_INTEGRATION_ID]
                sendCmd = ("?OUTPUT," + integration_id + ",1,")
                                
            elif dev.deviceTypeId == RA_CCO:
                cco = dev.pluginProps[PROP_INTEGRATION_ID]
                ccoType = dev.pluginProps[PROP_CCO_TYPE]
                if ccoType == "momentary":
                    self.logger.info(u"Momentary CCOs do not respond to Status Requests")
                else:
                    sendCmd = ("?OUTPUT," + cco + ",1,")

            elif dev.deviceTypeId == RA_CCI:
                self.logger.info(u"This device does not respond to Status Requests")

        if len(sendCmd):
            self._sendCommand(sendCmd)
            self.logger.debug(u"actionControlDimmerRelay sent: \"%s\" %s %s" % (dev.name, dev.onState, sendCmd))

    ######################
    # Sensor Action callback
    ######################
    def actionControlSensor(self, action, dev):
        self.logger.info(u"This device does not respond to Status Requests")

    ######################
    # Fan Action callback
    ######################
    def actionControlSpeedControl(self, action, dev):

        sendCmd = ""

        ###### SET SPEED ######
        if action.speedControlAction == indigo.kSpeedControlAction.SetSpeedIndex:
            if dev.deviceTypeId == RA_FAN:
                newSpeed = action.actionValue
                fan = dev.pluginProps[PROP_INTEGRATION_ID]
                self.logger.debug(u"{}: New speedIndex = {}".format(dev.name, newSpeed))
                if newSpeed == 0:
                    sendCmd = "#OUTPUT," + fan + ",1,0"
                elif newSpeed == 1:
                    sendCmd = "#OUTPUT," + fan + ",1,25"
                elif newSpeed == 2:
                    sendCmd = "#OUTPUT," + fan + ",1,75"
                elif newSpeed == 3:
                    sendCmd = "#OUTPUT," + fan + ",1,100"
                else
                    self.logger.error(u"{}: Invalid speedIndex = {}".format(dev.name, newSpeed)

        ###### CYCLE SPEED ######
        elif action.speedControlAction == indigo.kSpeedControlAction.cycleSpeedControlState:
            if dev.deviceTypeId == RA_FAN:
                speed = (dev.pluginProps['actualSpeed'] + 25) % 125    # 0 -> 25 -> 50 -> 75 -> 100 -> 0
                self.logger.debug(u"{}: New speed = {}".format(dev.name, speed))
                sendCmd = "#OUTPUT,{},1,{}".format(dev.pluginProps[PROP_INTEGRATION_ID], speed)

        ###### TOGGLE ######
        elif action.speedControlAction == indigo.kSpeedControlAction.toggle:
            fan = dev.pluginProps[PROP_INTEGRATION_ID]
            if dev.speedIndex:
                self.update_device_property(dev, "last_speed", dev.pluginProps['actualSpeed'])
                self.logger.debug(u"{}:Toggling off".format(dev.name, speed))
                sendCmd = "#OUTPUT,{},1,0".format(fan)
            else:
                self.logger.debug(u"{}: Toggling on, speed = {}".format(dev.name, dev.pluginProps["last_speed"]))
                sendCmd = "#OUTPUT,{},1,{}".format(fan, dev.pluginProps["last_speed"])

        ###### STATUS REQUEST ######
        elif action.speedControlAction == indigo.kSpeedControlAction.RequestStatus:
            integration_id = dev.pluginProps[PROP_INTEGRATION_ID]
            sendCmd = "?OUTPUT," + integration_id + ",1,"

        if len(sendCmd):
            self._sendCommand(sendCmd)
            self.logger.debug(u"actionControlSpeedControl sent: \"%s\" %s %s" % (dev.name, dev.onState, sendCmd))


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
            sendCmd = "?HVAC," + integration_id + ",1," # get temperature
            self._sendCommand(sendCmd)
            self.logger.debug(u"actionControlThermostat sent: \"%s\" %s" % (dev.name, sendCmd))
            
            sendCmd = "?HVAC," + integration_id + ",2," # get heat and cool setpoints
            self._sendCommand(sendCmd)
            self.logger.debug(u"actionControlThermostat sent: \"%s\" %s" % (dev.name, sendCmd))
            
            sendCmd = "?HVAC," + integration_id + ",3," # get operating mode
            self._sendCommand(sendCmd)
            self.logger.debug(u"actionControlThermostat sent: \"%s\" %s" % (dev.name, sendCmd))

            sendCmd = "?HVAC," + integration_id + ",4," # get fan mode

        if len(sendCmd):
            self._sendCommand(sendCmd)
            self.logger.debug(u"actionControlThermostat sent: \"%s\" %s" % (dev.name, sendCmd))



    ########################################
    # Plugin Actions object callbacks (pluginAction is an Indigo plugin action instance)

    def setFanSpeed(self, pluginAction, fanDevice):

        fanSpeed =  pluginAction.props["fanSpeed"]
        sendCmd = "#OUTPUT,{},1,25".format(fanDevice.address, fanSpeed)
        self.logger.debug(u"{}: Setting fan speed to {}".format(fanDevice.name, fanSpeed))
        self._sendCommand(sendCmd)

    def fadeDimmer(self, pluginAction, dimmerDevice):

        brightness =  indigo.activePlugin.substitute(pluginAction.props["brightness"])
        fadeTime =  indigo.activePlugin.substitute(pluginAction.props["fadeTime"])
        zone = dimmerDevice.address

        sendCmd = ("#OUTPUT," + zone + ",1," + str(brightness) + "," + str(fadeTime))
        self.logger.info(u"{}:  Set brightness to %s with fade %s".format(dimmerDevice.name, brightness, fadeTime))
        self._sendCommand(sendCmd)

    def sendRawCommand(self, pluginAction):

        sendCmd =  indigo.activePlugin.substitute(pluginAction.props["commandString"])
        self.logger.debug(u"Sending Raw Command: \"%s\"" % sendCmd)
        self._sendCommand(sendCmd)
        
    ########################################

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
            
                address = "{}.{}".format(device["ID"], button["Number"])
                name = u"{} - {} ({})".format(areaName, device["Name"], address)
                props = {
                    PROP_ROOM : areaName, 
                    PROP_LIST_TYPE : "button", 
                    PROP_INTEGRATION_ID : str(device["ID"]), 
                    PROP_COMPONENT_ID : str(button["Number"]), 
                    PROP_BUTTONTYPE : "Unknown",
                    PROP_ISBUTTON : "True"
                }
                self.createLutronDevice(RA_PICO, name, address, props, areaName)                    
           
        for zone in casetaData["LIPIdList"]["Zones"]:
            self.logger.info(u"Caseta Zone '{}' ({}), Area = {}".format(zone["Name"], zone["ID"], zone["Area"]["Name"]))

            try:
                areaName = zone["Area"]["Name"]
            except:
                areaname = u"Unknown"
                
            name = u"{} - {} ({})".format(areaName, zone["Name"], zone["ID"])
            props = {
                PROP_ROOM : areaName, 
                PROP_INTEGRATION_ID : zone["ID"],
                PROP_OUTPUTTYPE: "AUTO_DETECT"
            }
            self.createLutronDevice(RA_DIMMER, name, zone["ID"], props, areaName)
   
        self.logger.info(u"Creating Devices done.")        
        self.threadLock.release()
        return


    def createRRA2DevicesMenu(self, valuesDict, typeId):

        if not self.IP:
            self.logger.warning(u"Unable to create devices, no IP connection to repeater.")
            return False
            
        deviceThread = threading.Thread(target = self.createRRA2Devices, args = (valuesDict, ))
        deviceThread.start()    
        return True        

    def createRRA2Devices(self, valuesDict):
        
        if not self.threadLock.acquire(False):
            self.logger.warning(u"Unable to create devices, process already running.")
            return

        # set up variables based on options selected
        
        self.group_by = valuesDict["group_by"]
        self.create_unused_keypad = bool(valuesDict["create_unused_keypad"])
        self.create_unused_phantom = bool(valuesDict["create_unused_phantom"])
        self.create_event_triggers = bool(valuesDict["create_event_triggers"])
        self.create_group_triggers = bool(valuesDict["create_group_triggers"])

        if bool(valuesDict["use_local"]):
            xmlFile = os.path.expanduser(valuesDict["xmlFileName"])         
            self.logger.info(u"Creating Devices from file: %s, Grouping = %s, Create unprogrammed keypad buttons = %s, Create unprogrammed phantom buttons = %s" % \
                (xmlFile, self.group_by, self.create_unused_keypad, self.create_unused_phantom))
            try:
                root = ET.parse(xmlFile).getroot()
            except:
                self.logger.error(u"Unable to parse XML file: {}".format(xmlFile))
                self.threadLock.release()
                return
                
            self.logger.info(u"Creating Devices file read completed, parsing data...")

        else:
            ip_address = self.pluginPrefs["ip_address"]
            self.logger.info(u"Creating RRA2 Devices from repeater at %s, Grouping = %s, Create unprogrammed keypad buttons = %s, Create unprogrammed phantom buttons = %s" % \
                (ip_address, self.group_by, self.create_unused_keypad, self.create_unused_phantom))
            self.logger.info(u"Creating Devices - starting data fetch...")
            try:
                s = requests.Session()
                r = s.get('http://' + ip_address + '/login?login=lutron&password=lutron')
                r = s.get('http://' + ip_address + '/DbXmlInfo.xml')
                root = ET.fromstring(r.text)        
            except:
                self.logger.error(u"Unable to parse XML data from repeater.")
                self.threadLock.release()
                return

            self.logger.info(u"Creating Devices fetch completed, parsing data...")
        
        # iterate through parts of the XML data, 'Areas' first
        
        for room in root.findall('Areas/Area/Areas/Area'):
            self.logger.info("Finding devices in '%s'" % (room.attrib['Name']))
        
            for device in room.findall('DeviceGroups/Device'):
                self.logger.debug("\tDevice: %s (%s,%s)" % (device.attrib['Name'], device.attrib['IntegrationID'], device.attrib['DeviceType']))
                if device.attrib['DeviceType'] == "MAIN_REPEATER":
                    for component in device.findall('Components/Component'):
                        self.logger.debug("\t\tComponent: %s (%s)" % (component.attrib['ComponentNumber'], component.attrib['ComponentType']))
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
                            address = device.attrib['IntegrationID'] + "." + button

                            try:
                                buttonType = component.find("Button").attrib[PROP_BUTTONTYPE]
                            except:
                                buttonType = "Unknown"
                            props = {
                                PROP_ROOM : room.attrib['Name'], 
                                PROP_INTEGRATION_ID : device.attrib['IntegrationID'], 
                                PROP_COMPONENT_ID : button, 
                                PROP_BUTTONTYPE : buttonType,
                                PROP_ISBUTTON : "True"
                            }
                            self.createLutronDevice(RA_PHANTOM_BUTTON, name, address, props, room.attrib['Name'])

                        elif component.attrib['ComponentType'] == "LED":    # ignore LEDs for phantom buttons
                            pass

                        else:
                            self.logger.error("Unknown Component Type: %s (%s)" % (component.attrib['Name'], component.attrib['ComponentType']))

                else:
                    self.logger.error("Unknown Device Type: %s (%s)" % (device.attrib['Name'], device.attrib['DeviceType']))
                    
            for output in room.findall('Outputs/Output'):
                self.logger.debug("\tOutput: %s (%s) %s" % (output.attrib['Name'], output.attrib['IntegrationID'], output.attrib['OutputType']))

                if output.attrib['OutputType'] == "INC" or output.attrib['OutputType'] == "MLV" or output.attrib['OutputType'] == "AUTO_DETECT":
                    name = u"{} - Dimmer {} - {}".format(room.attrib['Name'], output.attrib['IntegrationID'], output.attrib['Name'])
                    props = {
                        PROP_ROOM : room.attrib['Name'], 
                        PROP_INTEGRATION_ID : output.attrib['IntegrationID'],
                        PROP_OUTPUTTYPE: output.attrib[PROP_OUTPUTTYPE],
                    }
                    self.createLutronDevice(RA_DIMMER, name, output.attrib['IntegrationID'], props, room.attrib['Name'])
                    
                elif output.attrib['OutputType'] == "NON_DIM":
                    name = u"{} - Switch {} - {}".format(room.attrib['Name'], output.attrib['IntegrationID'], output.attrib['Name'])
                    props = {
                        PROP_ROOM : room.attrib['Name'], 
                        PROP_INTEGRATION_ID : output.attrib['IntegrationID'],
                        PROP_OUTPUTTYPE: output.attrib[PROP_OUTPUTTYPE]
                    }
                    self.createLutronDevice(RA_SWITCH, name, output.attrib['IntegrationID'], props, room.attrib['Name'])
                        
                elif output.attrib['OutputType'] == "SYSTEM_SHADE":
                    name = u"{} - Shade {} - {}".format(room.attrib['Name'], output.attrib['IntegrationID'], output.attrib['Name'])
                    props = {
                        PROP_ROOM : room.attrib['Name'], 
                        PROP_INTEGRATION_ID : output.attrib['IntegrationID'],
                        PROP_OUTPUTTYPE: output.attrib[PROP_OUTPUTTYPE]
                    }
                    self.createLutronDevice(RA_SHADE, name, output.attrib['IntegrationID'], props, room.attrib['Name'])

                elif output.attrib['OutputType'] == "CEILING_FAN_TYPE":
                    name = u"{} - Fan {} - {}".format(room.attrib['Name'], output.attrib['IntegrationID'], output.attrib['Name'])
                    props = {
                        PROP_ROOM : room.attrib['Name'], 
                        PROP_INTEGRATION_ID : output.attrib['IntegrationID'],
                        PROP_OUTPUTTYPE: output.attrib[PROP_OUTPUTTYPE]
                    }
                    self.createLutronDevice(RA_FAN, name, output.attrib['IntegrationID'], props, room.attrib['Name'])

                elif output.attrib['OutputType'] == "CCO_PULSED":
                    name = u"{} - VCRX CCO Momentary {} - {}".format(room.attrib['Name'], output.attrib['IntegrationID'], output.attrib['Name'])
                    props = {
                        PROP_ROOM : room.attrib['Name'], 
                        PROP_INTEGRATION_ID : output.attrib['IntegrationID'], 
                        PROP_CCO_TYPE : "momentary", 
                        PROP_SUPPORTS_STATUS_REQUEST : "False",
                        PROP_OUTPUTTYPE: output.attrib[PROP_OUTPUTTYPE]
                    }
                    self.createLutronDevice(RA_CCO, name, output.attrib['IntegrationID'], props, room.attrib['Name'])

                elif output.attrib['OutputType'] == "CCO_MAINTAINED":
                    name = u"{} - VCRX CCO Sustained {} - {}".format(room.attrib['Name'], output.attrib['IntegrationID'], output.attrib['Name'])
                    props = {
                        PROP_ROOM : room.attrib['Name'], 
                        PROP_INTEGRATION_ID : output.attrib['IntegrationID'], 
                        PROP_CCO_TYPE : "sustained", 
                        PROP_SUPPORTS_STATUS_REQUEST : "True",
                        PROP_OUTPUTTYPE: output.attrib[PROP_OUTPUTTYPE]
                    }
                    self.createLutronDevice(RA_CCO, name, output.attrib['IntegrationID'], props, room.attrib['Name'])

                elif output.attrib['OutputType'] == "HVAC":
                    pass

                else:
                    self.logger.error("Unknown Output Type: {} ({}, {})".format(output.attrib['Name'], output.attrib['OutputType'], output.attrib['IntegrationID']))


            for device in room.findall('DeviceGroups/DeviceGroup/Devices/Device'):
                self.logger.debug("\tDevice: %s (%s,%s)" % (device.attrib['Name'], device.attrib['IntegrationID'], device.attrib['DeviceType']))

                if device.attrib['DeviceType'] == "SEETOUCH_KEYPAD" or device.attrib['DeviceType'] == "HYBRID_SEETOUCH_KEYPAD" or device.attrib['DeviceType'] == "SEETOUCH_TABLETOP_KEYPAD":  
                    for component in device.findall('Components/Component'):
                        self.logger.debug("\t\tComponent: %s (%s)" % (component.attrib['ComponentNumber'], component.attrib['ComponentType']))
                        if component.attrib['ComponentType'] == "BUTTON":

                            assignments = len(component.findall('Button/Actions/Action/Presets/Preset/PresetAssignments/PresetAssignment'))  
                            if not self.create_unused_keypad and assignments == 0:
                                continue
                            
                            address = device.attrib['IntegrationID'] + "." + component.attrib['ComponentNumber']
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
                                PROP_INTEGRATION_ID : device.attrib['IntegrationID'], 
                                PROP_COMPONENT_ID : component.attrib['ComponentNumber'], 
                                PROP_KEYPADBUT_DISPLAY_LED_STATE : "false", 
                                PROP_BUTTONTYPE : buttonType,
                               PROP_ISBUTTON : "True"
                            }
                            self.createLutronDevice(RA_KEYPAD, name, address, props, room.attrib['Name'])
                    
                            # create button LED, if needed for the button
                            
                            if ((keypadType == "SEETOUCH_KEYPAD") or (keypadType == "HYBRID_SEETOUCH_KEYPAD")) and (buttonNum > 6):
                                continue
                            if (keypadType == "SEETOUCH_TABLETOP_KEYPAD") and (buttonNum > 17):
                                continue
                                            
                            name = name + " LED"  
                            keypadLED = str(int(component.attrib['ComponentNumber']) + 80)
                            address = device.attrib['IntegrationID'] + "." + keypadLED
                            props = {
                                PROP_ROOM : room.attrib['Name'], 
                                PROP_LIST_TYPE : "LED", 
                                PROP_INTEGRATION_ID : device.attrib['IntegrationID'], 
                                PROP_COMPONENT_ID : keypadLED, 
                                PROP_KEYPADBUT_DISPLAY_LED_STATE : "False" 
                            }
                            self.createLutronDevice(RA_KEYPAD, name, address, props, room.attrib['Name'])

                        elif component.attrib['ComponentType'] == "LED":
                            pass    # LED device created same time as button
                    
                        else:
                            self.logger.error("Unknown Component Type: %s (%s)" % (component.attrib['Name'], component.attrib['ComponentType']))
                                         
                elif device.attrib['DeviceType'] == "VISOR_CONTROL_RECEIVER":
                    for component in device.findall('Components/Component'):
                        self.logger.debug("\t\tComponent: %s (%s)" % (component.attrib['ComponentNumber'], component.attrib['ComponentType']))
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
                            address = device.attrib['IntegrationID'] + "." + component.attrib['ComponentNumber']

                            try:
                                buttonType = component.find("Button").attrib[PROP_BUTTONTYPE]
                            except:
                                buttonType = u"Unknown"
                            props = {
                                PROP_ROOM : room.attrib['Name'], 
                                PROP_LIST_TYPE : "button", 
                                PROP_INTEGRATION_ID : device.attrib['IntegrationID'], 
                                PROP_COMPONENT_ID : component.attrib['ComponentNumber'], 
                                PROP_KEYPADBUT_DISPLAY_LED_STATE : "false", 
                                PROP_BUTTONTYPE : buttonType,
                                PROP_ISBUTTON : "True"
                            }
                            self.createLutronDevice(RA_KEYPAD, name, address, props, room.attrib['Name'])
                                
                            # create button LED, if needed for the button

                            name = name + " LED"  
                            keypadLED = str(int(component.attrib['ComponentNumber']) + 80)
                            address = device.attrib['IntegrationID'] + "." + keypadLED
                            props = {
                                PROP_ROOM : room.attrib['Name'], 
                                PROP_LIST_TYPE : "LED", 
                                PROP_INTEGRATION_ID : device.attrib['IntegrationID'], 
                                PROP_COMPONENT_ID : keypadLED, 
                                PROP_KEYPADBUT_DISPLAY_LED_STATE : "False" 
                            }
                            self.createLutronDevice(RA_KEYPAD, name, address, props, room.attrib['Name'])

                        elif component.attrib['ComponentType'] == "LED":
                            pass
                            
                        elif component.attrib['ComponentType'] == "CCI":
                            name = u"{} - VCRX CCI Input {:03}.{:02}".format(room.attrib['Name'], int(device.attrib['IntegrationID']), int(component.attrib['ComponentNumber']))
                            address = device.attrib['IntegrationID'] + "." + component.attrib['ComponentNumber']
                            props = {
                                PROP_ROOM : room.attrib['Name'], 
                                PROP_INTEGRATION_ID : device.attrib['IntegrationID'], 
                                PROP_COMPONENT_ID : component.attrib['ComponentNumber'], 
                                PROP_SUPPORTS_STATUS_REQUEST : "False" 
                            }
                            self.createLutronDevice(RA_CCI, name, address, props, room.attrib['Name'])

                        else:
                            self.logger.error("Unknown Component Type: %s (%s)" % (component.attrib['Name'], component.attrib['ComponentType']))
                                         
                elif device.attrib['DeviceType'] == "PICO_KEYPAD":
                    for component in device.findall('Components/Component'):
                        self.logger.debug("\t\tComponent: %s (%s)" % (component.attrib['ComponentNumber'], component.attrib['ComponentType']))
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
                            address = device.attrib['IntegrationID'] + "." + component.attrib['ComponentNumber']

                            try:
                                buttonType = component.find("Button").attrib[PROP_BUTTONTYPE]
                            except:
                                buttonType = u"Unknown"
                            props = {
                                PROP_ROOM : room.attrib['Name'], 
                                PROP_INTEGRATION_ID : device.attrib['IntegrationID'], 
                                PROP_COMPONENT_ID : component.attrib['ComponentNumber'], 
                                PROP_BUTTONTYPE : buttonType,
                                PROP_ISBUTTON : "True"
                            }
                            self.createLutronDevice(RA_PICO, name, address, props, room.attrib['Name'])

                        else:
                            self.logger.error("Unknown Component Type: %s (%s)" % (component.attrib['Name'], component.attrib['ComponentType']))
                                         
                elif device.attrib['DeviceType'] == "MOTION_SENSOR":
                    name = u"{} - Motion Sensor {}".format(room.attrib['Name'], device.attrib['IntegrationID'])
                    address = device.attrib['IntegrationID']
                    props = {
                        PROP_ROOM : room.attrib['Name'], 
                        PROP_INTEGRATION_ID : address, 
                        PROP_SUPPORTS_STATUS_REQUEST : 
                        "False" 
                    }
                    self.createLutronDevice(RA_SENSOR, name, address, props, room.attrib['Name'])
                    
                    # Create a Group (Room) device for every room that has a motion sensors
                    
                    name = u"Group {:03} - {}".format( int(room.attrib['IntegrationID']), room.attrib['Name'])
                    address = room.attrib['IntegrationID']
                    props = {
                        'group': address 
                    }
                    if not address in self.groups:
                        self.createLutronDevice(RA_GROUP, name, address, props, room.attrib['Name'])
                   
                    if self.create_group_triggers:
        
                        trigger_exists = False
                        for triggerId, trigger in self.triggers.iteritems():
                            if  (trigger.pluginTypeId == "groupEvent") and (trigger.pluginProps[PROP_GROUP] == address):
                                trigger_exists = True
                                break
            
                        if not trigger_exists:
                            if "Lutron" in indigo.triggers.folders:
                                theFolder = indigo.triggers.folders["Lutron"].id
                            else:
                                self.logger.debug("Creating Trigger Folder: '%s'" % ("Lutron"))            
                                theFolder = indigo.triggers.folder.create("Lutron").id
        
                            triggername = u"{} Occupied".format(name)
                            self.logger.info("Creating groupEvent trigger: '%s' (%s)" % (triggerName, address))
                            indigo.pluginEvent.create(name=triggerName, 
                                description="", 
                                folder=theFolder,
                                pluginId="com.jimandnoreen.indigoplugin.lutron-radiora2",
                                pluginTypeId="groupEvent",
                                props={PROP_GROUP : address, "occupancyPopUp": "3"}
                            )
                        
                            triggername = u"{} Unoccupied".format(name)
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
            address = PROP_EVENT + event.attrib['EventNumber']
            props = {
                PROP_EVENT: event.attrib['EventNumber']
            }
            self.createLutronDevice(RA_TIMECLOCKEVENT, name, address, props, "TimeClock")
            
            if self.create_event_triggers:
                trigger_exists = False
                for triggerId, trigger in self.triggers.iteritems():
                    if  (trigger.pluginTypeId == "timeClockEvent") and (trigger.pluginProps[PROP_EVENT] == event.attrib['EventNumber']):
                        trigger_exists = True
                        break
                
                if not trigger_exists:
                    if "Lutron" in indigo.triggers.folders:
                        theFolder = indigo.triggers.folders["Lutron"].id
                    else:
                        self.logger.debug("Creating Trigger Folder: '%s'" % ("Lutron"))            
                        theFolder = indigo.triggers.folder.create("Lutron").id
            
                    triggername = u"{} Trigger".format(name)
                    self.logger.info("Creating timeClockEvent trigger: '%s' (%s)" % (triggerName, event.attrib['EventNumber']))
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
            address = hvac.attrib['IntegrationID']
            props = {
                'thermo': address
            }
            self.createLutronDevice(RA_THERMO, name, address, props, "HVAC")
                     
                        
        self.logger.info(u"Creating Devices done.")        
        self.threadLock.release()
        return


    def createLutronDevice(self, devType, name, address, props, room):

        folderNameDict = {
            RA_PHANTOM_BUTTON   : "Lutron Phantom Buttons",
            RA_DIMMER           : "Lutron Dimmers",
            RA_SWITCH           : "Lutron Switches",
            RA_KEYPAD           : "Lutron Keypads",
            RA_FAN              : "Lutron Fans",
            RA_SENSOR           : "Lutron Sensors",
            RA_THERMO           : "Lutron Thermostats",
            RA_CCO              : "Lutron Switches",
            RA_CCI              : "Lutron Sensors",
            RA_SHADE            : "Lutron Shades",
            RA_PICO             : "Lutron Keypads",
            RA_GROUP            : "Lutron Room Groups",
            RA_TIMECLOCKEVENT   : "Lutron Timeclock Events"
        }

        # first, make sure this device doesn't exist.  Unless I screwed up, the addresses should be unique
        # it would be more efficient to search through the internal device lists, but a pain to code.
        # If it does exist, update with the new properties
        
        for dev in indigo.devices.iter("self"):
            if dev.address == address:
                if dev.pluginProps.get(PROP_ROOM, None):
                    return dev
                else:
                    self.logger.debug("Adding ROOM property to device: '{}' ({})".format(name, address))            
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
            foldername = u"Lutron " + room
            if folderName in indigo.devices.folders:
                theFolder = indigo.devices.folders[folderName].id
            else:
                self.logger.debug("Creating Device Folder: '%s'" % (folderName))            
                theFolder = indigo.devices.folder.create(folderName).id
        else:
            foldername = u"DEVICES"
            theFolder = 0
                    
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

