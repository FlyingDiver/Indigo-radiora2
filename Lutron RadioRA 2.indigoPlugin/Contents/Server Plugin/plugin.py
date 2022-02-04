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
import select  # was getting errors on the select.error exception in runConcurrentThread
import logging
import json
import os
import requests
import xml.etree.ElementTree as ET  # noqa
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
PROP_SUPPORTS_BATTERY = "SupportsBatteryLevel"

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


########################################
class IPGateway:
    ########################################

    def __init__(self, dev):
        self.logger = logging.getLogger("Plugin.IPGateway")
        self.devId = dev.id
        self.connected = False
        self.connIP = None
        self.buffer = b''
        self.host = dev.pluginProps["host"]
        self.port = int(dev.pluginProps["port"])

        self.timeout = 35  # Under some conditions Smart Bridge Pro takes a long time to connect

        dev.updateStateOnServer(key="status", value="Disconnected")
        dev.updateStateImageOnServer(indigo.kStateImageSel.SensorOff)

    def start(self):
        device = indigo.devices[self.devId]
        self.logger.info(f"{device.name}: Running IP Start")

        try:
            self.logger.info(f"{device.name}: Connecting via IP to {self.host}:{self.port}")
            self.connIP = telnetlib.Telnet(self.host, self.port, self.timeout)
        except socket.timeout:
            self.logger.error(f"{device.name}: Unable to connect to Lutron gateway. Timed out.")
            device.updateStateImageOnServer(indigo.kStateImageSel.SensorTripped)
            return
        except Exception as e:
            self.logger.warning(f"{device.name}: Error connecting to Telnet socket: {e.message}")
            device.updateStateImageOnServer(indigo.kStateImageSel.SensorTripped)
            return

        txt = self.connIP.read_until(b' ', self.timeout)
        self.logger.debug(f"{device.name}: self.connIP.read: {txt}")

        if b'login' not in txt:
            self.logger.debug(f"{device.name}: No login prompt, unable to send username")
            device.updateStateImageOnServer(indigo.kStateImageSel.SensorTripped)
            return

        self.logger.debug(f"{device.name}: Sending username")
        self.connIP.write((device.pluginProps["username"] + "\r\n").encode('ascii'))

        txt = self.connIP.read_until(b' ', self.timeout)
        self.logger.debug(f"{device.name}: self.connIP.read: {txt}")
        if b'password' not in txt:
            self.logger.debug(f"{device.name}: No password prompt, unable to send password")
            device.updateStateImageOnServer(indigo.kStateImageSel.SensorTripped)
            return

        self.logger.debug(f"{device.name}: Sending password")
        self.connIP.write((device.pluginProps["password"] + "\r\n").encode('ascii'))

        self.logger.debug(f"{device.name}: Login process complete, connected")
        self.timeout = 5  # Reset the timeout to something reasonable
        self.connected = True
        device.updateStateOnServer(key="status", value="Connected")
        device.updateStateImageOnServer(indigo.kStateImageSel.SensorOn)

    def stop(self):
        device = indigo.devices[self.devId]
        self.logger.info(f"{device.name}: Running IP Stop")

        self.connected = False
        device.updateStateOnServer(key="status", value="Disconnected")
        device.updateStateImageOnServer(indigo.kStateImageSel.SensorOff)
        try:
            self.connIP.close()
        except (Exception,):
            pass
        self.connIP = None

    @property
    def poll(self):
        device = indigo.devices[self.devId]

        if not self.connected:
            self.start()

        try:
            data = self.connIP.read_eager()
        except EOFError as e:
            self.logger.error(f"{device.name}: EOFError: {e.message}")
            self.connected = False
            device.updateStateOnServer(key="status", value="Disconnected")
            device.updateStateImageOnServer(indigo.kStateImageSel.SensorTripped)
            return None
        except Exception as e:
            self.logger.error(f"{device.name}: Unknown Error: {e.message}")
            self.connected = False
            device.updateStateOnServer(key="status", value="Disconnected")
            device.updateStateImageOnServer(indigo.kStateImageSel.SensorTripped)
            return None

        if len(data):
            self.logger.threaddebug(f"{device.name}: {len(data)} characters read:\n{data}")

            # add new data to existing buffer in case a message got split across reads
            self.buffer += data

        # is there a complete line in the buffer?

        i = self.buffer.find(b'\n')
        if i == -1:
            return None

        self.logger.threaddebug(f"{device.name}: self.buffer = '{self.buffer}'")
        msg = self.buffer[:i]
        self.logger.threaddebug(f"{device.name}: msg = '{msg}'")
        self.buffer = self.buffer[i + 1:]
        self.logger.threaddebug(f"{device.name}: self.buffer = '{self.buffer}'")
        return msg.decode('ascii')

    def send(self, cmd):
        device = indigo.devices[self.devId]
        self.logger.debug(f"{device.name}: Sending: '{cmd}'")

        cmd = cmd + "\r\n"
        try:
            self.connIP.write(cmd.encode('ascii'))
        except Exception as e:
            self.logger.warning(f"{device.name}: Error sending IP command, resetting connection:  {e.message}")
            self.stop()

    def fetchXML(self):

        s = requests.Session()
        r = s.get('http://' + self.host + '/login?login=lutron&password=lutron')  # noqa
        r = s.get('http://' + self.host + '/DbXmlInfo.xml')  # noqa

        return r.text


########################################
class SerialGateway:
    ########################################

    def __init__(self, dev):
        self.logger = logging.getLogger("Plugin.SerialGateway")
        self.devId = dev.id
        self.connected = False
        dev.updateStateOnServer(key="status", value="Disconnected")
        dev.updateStateImageOnServer(indigo.kStateImageSel.SensorOff)

        self.connSerial = None
        self.command = ''

    def start(self):
        device = indigo.devices[self.devId]
        self.logger.info(f"{device.name}: Running Serial Start")

        serialUrl = indigo.activePlugin.getSerialPortUrl(device.pluginProps, u"serialPort")
        self.logger.info(f"{device.name}: Serial Port URL is: {serialUrl}")

        try:
            self.connSerial = indigo.activePlugin.openSerial(u"Lutron Gateway", serialUrl, 9600, stopbits=1, timeout=2, writeTimeout=1)
            if self.connSerial is None:
                self.logger.error(f"{self.dev.name}: Failed to open serial port")
                self.dev.updateStateOnServer(key="status", value="Failed")
                self.dev.updateStateImageOnServer(indigo.kStateImageSel.SensorTripped)
                return

        except Exception as e:
            self.logger.error(f"{self.dev.name}: Failed to open serial port")
            self.dev.updateStateOnServer(key="status", value="Failed")
            self.dev.updateStateImageOnServer(indigo.kStateImageSel.SensorTripped)
            return

        self.connected = True
        self.dev.updateStateOnServer(key="status", value="Connected")
        self.dev.updateStateImageOnServer(indigo.kStateImageSel.SensorOn)

        # Disable main repeater terminal prompt
        self.send("#MONITORING,12,2")

        # Enable main repeater HVAC monitoring
        self.send("#MONITORING,17,1")

        # Enable main repeater monitoring param 18 (undocumented but seems to be enabled by default for ethernet connections)
        self.send("#MONITORING,18,1")

    def stop(self):
        self.logger.debug(u"Serial stop called")
        if self.connected:
            self.connSerial.close()
            self.connected = False
        self.connSerial = None

    def poll(self):

        if not self.connected:
            self.start()

        try:
            s = self.connSerial.read()
        except (Exception,):
            self.logger.error(u"{}: Error reading from serial port".format(self.dev.name))
            self.dev.updateStateOnServer(key="status", value="Failed")
            self.dev.updateStateImageOnServer(indigo.kStateImageSel.SensorTripped)
            self.connected = False
            return

        if len(s) > 0:

            if s == '\r':  # RadioRA 2 messages are always terminated with CRLF
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
        except (Exception,):
            self.logLevel = logging.INFO
        self.indigo_log_handler.setLevel(self.logLevel)
        self.logger.debug(u"logLevel = {}".format(self.logLevel))
        self.pluginVersion = pluginVersion
        self.pluginId = pluginId
        self.pluginDisplayName = pluginDisplayName

        self.gateways = {}
        self.defaultGateway = None

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
        self.roomList = []

        self.eventTriggers = {}
        self.groupTriggers = {}
        self.buttonPressTriggers = {}
        self.buttonMultiPressTriggers = {}

        self.linkedDeviceList = {}

        self.lastKeyTime = time.time()
        self.lastKeyAddress = ""
        self.lastKeyTaps = 0
        self.newKeyPress = False

        # global Timeouts
        self.click_delay = float(self.pluginPrefs.get(u"click_delay", "1.0"))
        self.click_timeout = float(self.pluginPrefs.get(u"click_timeout", "0.5"))

        self.threadLock = threading.Lock()  # for background data fetch

    def startup(self):
        self.logger.info(f"Starting up {self.pluginDisplayName}")

        savedList = self.pluginPrefs.get(u"linkedDevices", None)
        if savedList:
            self.linkedDeviceList = json.loads(savedList)
            self.logLinkedDevices()

        indigo.devices.subscribeToChanges()

        ################################################################################
        # convert to device-based gateways
        ################################################################################

        converted = self.pluginPrefs.get(u"Converted", False)
        if converted:

            self.logger.debug("Previously converted, default gateway ID = {}".format(converted))
            self.defaultGateway = int(converted)

        elif self.pluginPrefs.get(u"IP", None) or self.pluginPrefs.get(u"serialPort_uiAddress", None):

            self.logger.info("Converting to multiple gateway system")

            if self.pluginPrefs.get(u"IP", None):
                address = "{}:{}".format(self.pluginPrefs["ip_address"], self.pluginPrefs["ip_port"])
                name = "Lutron IP Gateway"
                props = {
                    "host": self.pluginPrefs["ip_address"],
                    "port": self.pluginPrefs["ip_port"],
                    "username": self.pluginPrefs["ip_username"],
                    "password": self.pluginPrefs["ip_password"]
                }
                self.logger.info("Creating new IP Gateway device @ {}".format(address))
                try:
                    newDevice = indigo.device.create(indigo.kProtocol.Plugin, address=address, name=name, deviceTypeId=DEV_IP_GATEWAY, props=props)
                except Exception as e:
                    self.logger.error("Error in indigo.device.create(): {}".format(e.message))
                    return

                self.logger.info("IP Gateway device complete")

            else:
                address = self.pluginPrefs["serialPort_uiAddress"]
                name = "Lutron Serial Gateway"
                props = {
                    "serialPort_serialConnType": self.pluginPrefs["serialPort_serialConnType"],
                    "serialport_serialPortLocal": self.pluginPrefs["serialport_serialPortLocal"],
                    "serialPort_serialPortNetRfc2217": self.pluginPrefs["serialPort_serialPortNetRfc2217"],
                    "serialPort_serialPortNetSocket": self.pluginPrefs["serialPort_serialPortNetSocket"],
                    "serialPort_uiAddress": self.pluginPrefs["serialPort_uiAddress"]
                }
                self.logger.info("Creating new Serial Gateway device @ {}".format(address))
                try:
                    newDevice = indigo.device.create(indigo.kProtocol.Plugin, address=address, name=name, deviceTypeId=DEV_SERIAL_GATEWAY,
                                                     props=props)
                except Exception as e:
                    self.logger.error("Error in indigo.device.create(): {}".format(e.message))
                    return

                self.logger.info("Serial Gateway device complete")

            self.defaultGateway = newDevice.id
            self.pluginPrefs[u"Converted"] = str(self.defaultGateway)
            self.savePluginPrefs()

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
            if (delDevice.id == int(linkItem["buttonDevice"])) or (delDevice.id == int(linkItem["buttonLEDDevice"])) or (
                    delDevice.id == int(linkItem["controlledDevice"])):
                self.logger.info(u"A linked device ({}) has been deleted.  Deleting link: {}".format(delDevice.name, linkItem["name"]))
                del self.linkedDeviceList[linkID]
                self.logLinkedDevices()

                indigo.activePlugin.pluginPrefs[u"linkedDevices"] = json.dumps(self.linkedDeviceList)

    def deviceUpdated(self, oldDevice, newDevice):
        indigo.PluginBase.deviceUpdated(self, oldDevice, newDevice)

        for linkName, linkItem in self.linkedDeviceList.items():
            controlledDevice = indigo.devices[int(linkItem["controlledDevice"])]
            buttonDevice = indigo.devices[int(linkItem["buttonDevice"])]

            if oldDevice.id == controlledDevice.id:

                self.logger.debug(u"A linked device ({}) has been updated: {}".format(controlledDevice.name, controlledDevice.onState))
                try:
                    buttonLEDDevice = indigo.devices[int(linkItem["buttonLEDDevice"])]
                except (Exception,):
                    pass
                else:
                    if controlledDevice.onState:
                        indigo.device.turnOn(buttonLEDDevice.id)
                    else:
                        indigo.device.turnOff(buttonLEDDevice.id)

    ####################

    def triggerStartProcessing(self, trigger):
        self.logger.threaddebug("Starting Trigger '{}', pluginProps = {}".format(trigger.name, trigger.pluginProps))

        # based on trigger type, do property adjustments and add to trigger list

        if trigger.pluginTypeId == "keypadButtonPress":

            buttonID = trigger.pluginProps.get("buttonID", None)
            if not buttonID:
                self.logger.error(
                    "keypadButtonPress Trigger  {} ({}) missing buttonID: {}".format(trigger.name, trigger.id, str(trigger.pluginProps)))
                return

            self.logger.debug("Adding keypadButtonPress Trigger '{}', buttonID = {}".format(trigger.name, buttonID))
            self.buttonPressTriggers[trigger.id] = trigger

        elif trigger.pluginTypeId == "keypadMultiButtonPress":

            buttonID = trigger.pluginProps.get("buttonID", None)
            if not buttonID:
                self.logger.error(
                    "keypadMultiButtonPress Trigger  {} ({}) missing buttonID: {}".format(trigger.name, trigger.id, str(trigger.pluginProps)))
                return

            self.logger.debug("Adding keypadMultiButtonPress Trigger '{}', buttonID = {}".format(trigger.name, buttonID))
            self.buttonMultiPressTriggers[trigger.id] = trigger

        elif trigger.pluginTypeId == "timeClockEvent":

            event = trigger.pluginProps.get(PROP_EVENT, None)
            if not event:
                self.logger.error(u"timeClockEvent Trigger {} ({}) does not contain event: {}".format(trigger.name, trigger.id, trigger.pluginProps))
                return

            # add the default gateway if not already in props

            gateway = trigger.pluginProps.get(PROP_GATEWAY, None)
            if not gateway:
                self.update_plugin_property(trigger, PROP_GATEWAY, self.defaultGateway)
                self.logger.info(u"{}: Added default Gateway ({})".format(trigger.name, self.defaultGateway))

            self.logger.debug("Adding timeClockEvent Trigger {}, event = {}, gateway = {}".format(trigger.name, event, gateway))
            self.eventTriggers[trigger.id] = trigger

        elif trigger.pluginTypeId == "groupEvent":

            group = trigger.pluginProps.get(PROP_GROUP, None)
            if not group:
                self.logger.error(u"Group Trigger {} ({}) does not contain group: {}".format(trigger.name, trigger.id, trigger.pluginProps))
                return

            # add the default gateway if not already in props

            gateway = trigger.pluginProps.get(PROP_GATEWAY, None)
            if not gateway:
                self.update_plugin_property(trigger, PROP_GATEWAY, self.defaultGateway)
                self.logger.info(u"{}: Added default Gateway ({})".format(trigger.name, self.defaultGateway))

            self.logger.debug("Adding Group Trigger {}, group = {}, gateway = {}".format(trigger.name, group, gateway))
            self.groupTriggers[trigger.id] = trigger

        else:
            self.logger.error(u"triggerStartProcessing: Trigger {} ({}) is unknown type: {}".format(trigger.name, trigger.id, trigger.pluginTypeId))

    def triggerStopProcessing(self, trigger):

        self.logger.debug(u"Removing Trigger {} ({})".format(trigger.name, trigger.id))
        if trigger.pluginTypeId == "keypadButtonPress":
            del self.buttonPressTriggers[trigger.id]
        elif trigger.pluginTypeId == "keypadMultiButtonPress":
            del self.buttonMultiPressTriggers[trigger.id]
        elif trigger.pluginTypeId == "timeClockEvent":
            del self.eventTriggers[trigger.id]
        elif trigger.pluginTypeId == "groupEvent":
            del self.groupTriggers[trigger.id]
        else:
            self.logger.error(u"triggerStopProcessing: Trigger {} ({}) is unknown type: {}".format(trigger.name, trigger.id, trigger.pluginTypeId))

    def eventTriggerCheck(self, eventID, gatewayID):

        self.logger.debug(u"eventTriggerCheck: event {}, gateway: {}".format(eventID, gatewayID))

        for trigger in self.eventTriggers.values():

            if (eventID == trigger.pluginProps[PROP_EVENT]) and (gatewayID == trigger.pluginProps[PROP_GATEWAY]):
                self.logger.debug(u"eventTriggerCheck: Executing Trigger {} ({})".format(trigger.name, trigger.id))
                indigo.trigger.execute(trigger)
            else:
                self.logger.threaddebug(u"eventTriggerCheck: Skipping Trigger {} ({})".format(trigger.name, trigger.id))

    def groupTriggerCheck(self, groupID, gatewayID, status):

        self.logger.debug(u"groupTriggerCheck: group: {}, gateway: {}, status: {}".format(groupID, gatewayID, status))

        for trigger in self.groupTriggers.values():

            if (trigger.pluginProps[PROP_GROUP] == groupID) and (trigger.pluginProps[PROP_GATEWAY] == gatewayID) and (
                    trigger.pluginProps["occupancyPopUp"] == status):
                self.logger.debug(u"groupTriggerCheck: Executing Trigger {} ({})".format(trigger.name, trigger.id))
                indigo.trigger.execute(trigger)
            else:
                self.logger.threaddebug(u"groupTriggerCheck: Skipping Trigger {} ({})".format(trigger.name, trigger.id))

    def buttonTriggerCheck(self, deviceID, componentID, gatewayID):

        self.logger.debug(u"buttonTriggerCheck: deviceID: {}, componentID: {}, gatewayID: {}".format(deviceID, componentID, gatewayID))
        keypadid = "{}:{}.{}".format(gatewayID, deviceID, componentID)

        # check for linked devices

        for linkItem in self.linkedDeviceList.values():
            controlledDevice = indigo.devices[int(linkItem["controlledDevice"])]
            buttonAddress = linkItem["buttonAddress"]
            if buttonAddress == keypadid:
                self.logger.debug(u"Linked Device Match, buttonAddress: {}, controlledDevice: {}".format(buttonAddress, controlledDevice.id))
                indigo.device.toggle(controlledDevice.id)

        # and check for multiple taps

        if (keypadid == self.lastKeyAddress) and (time.time() < (self.lastKeyTime + self.click_delay)):
            self.lastKeyTaps += 1
        else:
            self.lastKeyTaps = 1

        self.lastKeyAddress = keypadid
        self.lastKeyTime = time.time()
        self.newKeyPress = True

        # Look for old-style triggers that match this button

        for trigger in self.buttonPressTriggers.values():

            buttonID = trigger.pluginProps["buttonID"]
            try:
                buttonAddress = indigo.devices[int(buttonID)].address
            except(Exception, ):
                self.logger.error(u"buttonTriggerCheck: invalid or missing buttonID {} in trigger '{}'".format(buttonID, trigger.name))
                return

            if keypadid != buttonAddress:
                self.logger.threaddebug(u"buttonTriggerCheck: Skipping Trigger '{}', wrong keypad button: {}".format(trigger.name, keypadid))
                continue

            clicks = int(trigger.pluginProps.get("clicks", "1"))
            if self.lastKeyTaps != int(trigger.pluginProps["clicks"]):
                self.logger.threaddebug(u"buttonTriggerCheck: Skipping Trigger {}, wrong click count: {}".format(trigger.name, self.lastKeyTaps))
                continue

            self.logger.debug(u"buttonTriggerCheck: Executing Trigger '{}', keypad button: ".format(trigger.name, buttonAddress))
            indigo.trigger.execute(trigger)

    # called from the main run look to process queued keypresses for triggers    

    def buttonMultiPressCheck(self):

        if self.newKeyPress:

            # if last key press hasn't timed out yet, don't do anything
            if time.time() < (self.lastKeyTime + self.click_timeout):
                return

            self.logger.debug(u"buttonMultiPressCheck: Timeout reached for keypadid = {}, presses = {}".format(self.lastKeyAddress, self.lastKeyTaps))

            # Look for new-style triggers that match this button

            for trigger in self.buttonMultiPressTriggers.values():

                buttonID = trigger.pluginProps["buttonID"]
                try:
                    buttonAddress = indigo.devices[int(buttonID)].address
                except(Exception,):
                    self.logger.error(u"buttonMultiPressCheck: invalid or missing buttonID {} in trigger '{}'".format(buttonID, trigger.name))
                    return

                if buttonAddress != self.lastKeyAddress:
                    self.logger.threaddebug(
                        u"buttonMultiPressCheck: Skipping Trigger '{}', wrong keypad button: {}".format(trigger.name, self.lastKeyAddress))
                    continue

                clicks = int(trigger.pluginProps.get("clicks", "1"))
                if self.lastKeyTaps != int(trigger.pluginProps["clicks"]):
                    self.logger.threaddebug(
                        u"buttonMultiPressCheck: Skipping Trigger {}, wrong click count: {}".format(trigger.name, self.lastKeyTaps))
                    continue

                self.logger.debug(u"buttonMultiPressCheck: Executing Trigger '{}', keypad button: {}".format(trigger.name, self.lastKeyAddress))
                indigo.trigger.execute(trigger)

            # all done, reset the flag
            self.newKeyPress = False

    ####################

    def update_plugin_property(self, obj, propertyname, new_value=""):
        newProps = obj.pluginProps
        newProps.update({propertyname: new_value})
        obj.replacePluginPropsOnServer(newProps)
        self.sleep(0.01)
        return None

    def remove_plugin_property(self, obj, propertyname):
        newProps = obj.pluginProps
        del newProps[propertyname]
        obj.replacePluginPropsOnServer(newProps)
        self.sleep(0.01)
        return None

    def deviceStartComm(self, dev):

        if dev.deviceTypeId == DEV_IP_GATEWAY:
            gateway = IPGateway(dev)
            self.gateways[dev.id] = gateway
            dev.updateStateOnServer(key="status", value="None")
            dev.updateStateImageOnServer(indigo.kStateImageSel.SensorOff)
            gateway.start()

        elif dev.deviceTypeId == DEV_SERIAL_GATEWAY:
            gateway = SerialGateway(dev)
            self.gateways[dev.id] = gateway
            dev.updateStateOnServer(key="status", value="None")
            dev.updateStateImageOnServer(indigo.kStateImageSel.SensorOff)
            gateway.start()

        elif dev.deviceTypeId == DEV_PHANTOM_BUTTON:
            if dev.pluginProps.get(PROP_REPEATER, None):
                self.update_plugin_property(dev, PROP_INTEGRATION_ID, dev.pluginProps[PROP_REPEATER])
                self.remove_plugin_property(dev, PROP_REPEATER)
                self.logger.info(u"{}: Updated repeater property to IntegrationID".format(dev.name))
            elif dev.pluginProps.get(PROP_INTEGRATION_ID, None) == None:
                self.update_plugin_property(dev, PROP_INTEGRATION_ID, "1")
                self.logger.info(u"{}: Added IntegrationID property".format(dev.name))

            if dev.pluginProps.get(PROP_BUTTON, None):
                self.update_plugin_property(dev, PROP_COMPONENT_ID, dev.pluginProps[PROP_BUTTON])
                self.remove_plugin_property(dev, PROP_BUTTON)
                self.logger.info(u"{}: Updated button property to componentID".format(dev.name))

            if dev.pluginProps.get(PROP_ISBUTTON, None) == None:
                self.update_plugin_property(dev, PROP_ISBUTTON, "True")
                self.logger.info(u"{}: Added isButton property".format(dev.name))

            if dev.pluginProps.get(PROP_GATEWAY, None) == None:
                self.update_plugin_property(dev, PROP_GATEWAY, self.defaultGateway)
                self.logger.info(u"{}: Added Gateway property".format(dev.name))

            address = u"{}:{}.{}".format(dev.pluginProps[PROP_GATEWAY], dev.pluginProps[PROP_INTEGRATION_ID], dev.pluginProps[PROP_COMPONENT_ID])
            self.phantomButtons[address] = dev
            if dev.address != address:
                self.update_plugin_property(dev, "address", address)

        elif dev.deviceTypeId == DEV_DIMMER:
            if dev.pluginProps.get(PROP_INTEGRATION_ID, None) == None:
                self.update_plugin_property(dev, PROP_INTEGRATION_ID, dev.pluginProps[PROP_ZONE])
                self.remove_plugin_property(dev, PROP_ZONE)
                self.logger.info(u"{}: Updated zone property to IntegrationID".format(dev.name))

            if dev.pluginProps.get(PROP_GATEWAY, None) == None:
                self.update_plugin_property(dev, PROP_GATEWAY, self.defaultGateway)
                self.logger.info(u"{}: Added Gateway property".format(dev.name))

            address = u"{}:{}".format(dev.pluginProps[PROP_GATEWAY], dev.pluginProps[PROP_INTEGRATION_ID])
            self.dimmers[address] = dev
            if dev.address != address:
                self.update_plugin_property(dev, "address", address)

        elif dev.deviceTypeId == DEV_SHADE:
            if dev.pluginProps.get(PROP_INTEGRATION_ID, None) == None:
                self.update_plugin_property(dev, PROP_INTEGRATION_ID, dev.pluginProps[PROP_SHADE])
                self.remove_plugin_property(dev, PROP_SHADE)
                self.logger.info(u"{}: Updated zone property to IntegrationID".format(dev.name))

            if dev.pluginProps.get(PROP_GATEWAY, None) == None:
                self.update_plugin_property(dev, PROP_GATEWAY, self.defaultGateway)
                self.logger.info(u"{}: Added Gateway property".format(dev.name))

            address = u"{}:{}".format(dev.pluginProps[PROP_GATEWAY], dev.pluginProps[PROP_INTEGRATION_ID])
            self.shades[address] = dev
            dev.updateStateImageOnServer(indigo.kStateImageSel.NoImage)
            if dev.address != address:
                self.update_plugin_property(dev, "address", address)

        elif dev.deviceTypeId == DEV_SWITCH:
            if dev.pluginProps.get(PROP_INTEGRATION_ID, None) == None:
                self.update_plugin_property(dev, PROP_INTEGRATION_ID, dev.pluginProps[PROP_SWITCH])
                self.remove_plugin_property(dev, PROP_SWITCH)
                self.logger.info(u"{}: Updated switch property to IntegrationID".format(dev.name))

            if dev.pluginProps.get(PROP_GATEWAY, None) == None:
                self.update_plugin_property(dev, PROP_GATEWAY, self.defaultGateway)
                self.logger.info(u"{}: Added Gateway property".format(dev.name))

            address = u"{}:{}".format(dev.pluginProps[PROP_GATEWAY], dev.pluginProps[PROP_INTEGRATION_ID])
            self.switches[address] = dev
            if dev.address != address:
                self.update_plugin_property(dev, "address", address)

        elif dev.deviceTypeId == DEV_FAN:
            if dev.pluginProps.get(PROP_INTEGRATION_ID, None) == None:
                self.update_plugin_property(dev, PROP_INTEGRATION_ID, dev.pluginProps[PROP_FAN])
                self.remove_plugin_property(dev, PROP_FAN)
                self.logger.info(u"{}: Updated fan property to IntegrationID".format(dev.name))

            if dev.pluginProps.get(PROP_GATEWAY, None) == None:
                self.update_plugin_property(dev, PROP_GATEWAY, self.defaultGateway)
                self.logger.info(u"{}: Added Gateway property".format(dev.name))

            address = u"{}:{}".format(dev.pluginProps[PROP_GATEWAY], dev.pluginProps[PROP_INTEGRATION_ID])
            self.fans[address] = dev
            if dev.address != address:
                self.update_plugin_property(dev, "address", address)

        elif dev.deviceTypeId == DEV_THERMO:
            if dev.pluginProps.get(PROP_INTEGRATION_ID, None) == None:
                self.update_plugin_property(dev, PROP_INTEGRATION_ID, dev.pluginProps[PROP_THERMO])
                self.remove_plugin_property(dev, PROP_THERMO)
                self.logger.info(u"{}: Updated thermo property to IntegrationID".format(dev.name))

            if dev.pluginProps.get(PROP_GATEWAY, None) == None:
                self.update_plugin_property(dev, PROP_GATEWAY, self.defaultGateway)
                self.logger.info(u"{}: Added Gateway property".format(dev.name))

            address = u"{}:{}".format(dev.pluginProps[PROP_GATEWAY], dev.pluginProps[PROP_INTEGRATION_ID])
            self.thermos[address] = dev
            if dev.address != address:
                self.update_plugin_property(dev, "address", address)

        elif dev.deviceTypeId == DEV_KEYPAD:
            if dev.pluginProps.get(PROP_INTEGRATION_ID, None) == None:
                self.update_plugin_property(dev, PROP_INTEGRATION_ID, dev.pluginProps[PROP_KEYPAD])
                self.remove_plugin_property(dev, PROP_KEYPAD)
                self.logger.info(u"{}: Updated keypad property to IntegrationID".format(dev.name))

            if dev.pluginProps.get(PROP_KEYPADBUT, None):
                self.update_plugin_property(dev, PROP_COMPONENT_ID, dev.pluginProps[PROP_KEYPADBUT])
                self.remove_plugin_property(dev, PROP_KEYPADBUT)
                self.logger.info(u"{}: Updated keypadButton property to componentID".format(dev.name))

            if (dev.pluginProps.get(PROP_ISBUTTON, None) == None) and (int(dev.pluginProps[PROP_COMPONENT_ID]) < 80):
                self.update_plugin_property(dev, PROP_ISBUTTON, new_value="True")
                self.logger.info(u"{}: Added isButton property".format(dev.name))

            if dev.pluginProps.get(PROP_GATEWAY, None) == None:
                self.update_plugin_property(dev, PROP_GATEWAY, self.defaultGateway)
                self.logger.info(u"{}: Added Gateway property".format(dev.name))

            address = u"{}:{}.{}".format(dev.pluginProps[PROP_GATEWAY], dev.pluginProps[PROP_INTEGRATION_ID], dev.pluginProps[PROP_COMPONENT_ID])
            self.keypads[address] = dev.id
            if int(dev.pluginProps[PROP_COMPONENT_ID]) > 80:
                self.update_plugin_property(dev, PROP_KEYPADBUT_DISPLAY_LED_STATE, new_value=dev.pluginProps[PROP_KEYPADBUT_DISPLAY_LED_STATE])
            else:
                self.update_plugin_property(dev, PROP_KEYPADBUT_DISPLAY_LED_STATE, new_value="False")
            if dev.address != address:
                self.update_plugin_property(dev, "address", address)

        elif dev.deviceTypeId == DEV_SENSOR:
            if dev.pluginProps.get(PROP_INTEGRATION_ID, None) == None:
                self.update_plugin_property(dev, PROP_INTEGRATION_ID, dev.pluginProps[PROP_SENSOR])
                self.remove_plugin_property(dev, PROP_SENSOR)
                self.logger.info(u"{}: Updated sensor property to IntegrationID".format(dev.name))

            if dev.pluginProps.get(PROP_GATEWAY, None) == None:
                self.update_plugin_property(dev, PROP_GATEWAY, self.defaultGateway)
                self.logger.info(u"{}: Added Gateway property".format(dev.name))

            address = u"{}:{}".format(dev.pluginProps[PROP_GATEWAY], dev.pluginProps[PROP_INTEGRATION_ID])
            self.sensors[address] = dev
            if dev.address != address:
                self.update_plugin_property(dev, "address", address)

        elif dev.deviceTypeId == DEV_CCI:
            if dev.pluginProps.get(PROP_INTEGRATION_ID, None) == None:
                self.update_plugin_property(dev, PROP_INTEGRATION_ID, dev.pluginProps[PROP_CCI_INTEGRATION_ID])
                self.remove_plugin_property(dev, PROP_CCI_INTEGRATION_ID)
                self.logger.info(u"{}: Updated cciIntegrationID property to IntegrationID".format(dev.name))

            if dev.pluginProps.get(PROP_COMPONENT, None):
                self.update_plugin_property(dev, PROP_COMPONENT_ID, dev.pluginProps[PROP_COMPONENT])
                self.remove_plugin_property(dev, PROP_COMPONENT)
                self.logger.info(u"{}: Updated cciCompoment property to componentID".format(dev.name))

            if dev.pluginProps.get(PROP_GATEWAY, None) == None:
                self.update_plugin_property(dev, PROP_GATEWAY, self.defaultGateway)
                self.logger.info(u"{}: Added Gateway property".format(dev.name))

            address = u"{}:{}.{}".format(dev.pluginProps[PROP_GATEWAY], dev.pluginProps[PROP_INTEGRATION_ID], dev.pluginProps[PROP_COMPONENT_ID])
            self.ccis[address] = dev
            if dev.address != address:
                self.update_plugin_property(dev, "address", address)

        elif dev.deviceTypeId == DEV_CCO:
            if dev.pluginProps.get(PROP_INTEGRATION_ID, None) == None:
                self.update_plugin_property(dev, PROP_INTEGRATION_ID, dev.pluginProps[PROP_CCO_INTEGRATION_ID])
                self.remove_plugin_property(dev, PROP_CCO_INTEGRATION_ID)
                self.logger.info(u"{}: Updated ccoIntegrationID property to IntegrationID".format(dev.name))

            if dev.pluginProps.get(PROP_GATEWAY, None) == None:
                self.update_plugin_property(dev, PROP_GATEWAY, self.defaultGateway)
                self.logger.info(u"{}: Added Gateway property".format(dev.name))

            address = u"{}:{}".format(dev.pluginProps[PROP_GATEWAY], dev.pluginProps[PROP_INTEGRATION_ID])
            self.ccos[address] = dev
            ccoType = dev.pluginProps[PROP_CCO_TYPE]
            if ccoType == "momentary":
                dev.updateStateOnServer(ONOFF, False)
            else:
                self.update_plugin_property(dev, PROP_SUPPORTS_STATUS_REQUEST, new_value="True")
            if dev.address != address:
                self.update_plugin_property(dev, "address", address)

        elif dev.deviceTypeId == DEV_PICO:
            if dev.pluginProps.get(PROP_INTEGRATION_ID, None) == None:
                self.update_plugin_property(dev, PROP_INTEGRATION_ID, dev.pluginProps[PROP_PICO_INTEGRATION_ID])
                self.remove_plugin_property(dev, PROP_PICO_INTEGRATION_ID)
                self.logger.info(u"{}: Updated picoIntegrationID property to IntegrationID".format(dev.name))

            if dev.pluginProps.get(PROP_ISBUTTON, None) == None:
                self.update_plugin_property(dev, PROP_ISBUTTON, new_value="True")
                self.logger.info(u"{}: Added isButton property".format(dev.name))

            if dev.pluginProps.get(PROP_PICOBUTTON, None):
                self.update_plugin_property(dev, PROP_COMPONENT_ID, dev.pluginProps[PROP_PICOBUTTON])
                self.remove_plugin_property(dev, PROP_PICOBUTTON)
                self.logger.info(u"{}: Updated keypadButton property to componentID".format(dev.name))

            if dev.pluginProps.get(PROP_GATEWAY, None) == None:
                self.update_plugin_property(dev, PROP_GATEWAY, self.defaultGateway)
                self.logger.info(u"{}: Added Gateway property".format(dev.name))

            address = u"{}:{}.{}".format(dev.pluginProps[PROP_GATEWAY], dev.pluginProps[PROP_INTEGRATION_ID], dev.pluginProps[PROP_COMPONENT_ID])
            self.picos[address] = dev
            if dev.address != address:
                self.update_plugin_property(dev, "address", address)

        elif dev.deviceTypeId == DEV_TIMECLOCKEVENT:

            if dev.pluginProps.get(PROP_GATEWAY, None) == None:
                self.update_plugin_property(dev, PROP_GATEWAY, self.defaultGateway)
                self.logger.info(u"{}: Added Gateway property".format(dev.name))

            address = u"{}:Event.{}".format(dev.pluginProps[PROP_GATEWAY], dev.pluginProps[PROP_EVENT])
            self.events[address] = dev
            if dev.address != address:
                self.update_plugin_property(dev, "address", address)

        elif dev.deviceTypeId == DEV_GROUP:

            if dev.pluginProps.get(PROP_GATEWAY, None) == None:
                self.update_plugin_property(dev, PROP_GATEWAY, self.defaultGateway)
                self.logger.info(u"{}: Added Gateway property".format(dev.name))

            # fix device properties to show correct UI in Indigo client, matches Devices.xml
            newProps = dev.pluginProps
            newProps["SupportsOnState"] = True
            newProps["SupportsSensorValue"] = False
            newProps["SupportsStatusRequest"] = False

            address = u"{}:Group.{}".format(dev.pluginProps[PROP_GATEWAY], dev.pluginProps[PROP_GROUP])
            newProps["address"] = address
            dev.replacePluginPropsOnServer(newProps)
            self.groups[address] = dev

        elif dev.deviceTypeId == DEV_LINKEDDEVICE:

            # migrate devices then delete them

            buttonDeviceId = int(dev.pluginProps["buttonDevice"])
            buttonLEDDeviceId = int(dev.pluginProps["buttonLEDDevice"])
            controlledDeviceId = int(dev.pluginProps["controlledDevice"])
            buttonAddress = dev.pluginProps["buttonAddress"]

            linkID = "{}-{}".format(buttonDeviceId, controlledDeviceId)
            linkItem = {"name": linkID, "buttonDevice": buttonDeviceId, "buttonLEDDevice": buttonLEDDeviceId, "controlledDevice": controlledDeviceId,
                        "buttonAddress": buttonAddress}
            self.logger.debug(u"Adding linkItem {}: {}".format(linkID, linkItem))
            self.linkedDeviceList[linkID] = linkItem
            self.logLinkedDevices()
            indigo.activePlugin.pluginPrefs[u"linkedDevices"] = json.dumps(self.linkedDeviceList)
            indigo.device.delete(dev.id)

        else:
            self.logger.error(u"{}: deviceStartComm: Unknown device type: {}".format(dev.name, dev.deviceTypeId))
            return

        # Make sure we have a complete list of rooms that have button devices in them

        if (dev.pluginProps.get(PROP_ISBUTTON, None)):
            roomName = dev.pluginProps.get(PROP_ROOM, u"Unknown")
            if roomName not in self.roomList:
                self.roomList.append(roomName)

        dev.stateListOrDisplayStateIdChanged()

    def deviceStopComm(self, dev):

        try:
            if dev.deviceTypeId == DEV_IP_GATEWAY:

                gateway = self.gateways[dev.id]
                gateway.stop()
                dev.updateStateOnServer(key="status", value="None")
                dev.updateStateImageOnServer(indigo.kStateImageSel.SensorOff)
                del self.gateways[dev.id]

            elif dev.deviceTypeId == DEV_SERIAL_GATEWAY:

                gateway = self.gateways[dev.id]
                gateway.stop()
                dev.updateStateOnServer(key="status", value="None")
                dev.updateStateImageOnServer(indigo.kStateImageSel.SensorOff)
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

            else:
                self.logger.error(u"{}: deviceStopComm: Unknown device type: {}".format(dev.name, dev.deviceTypeId))
        except:
            pass

    def validateDeviceConfigUi(self, valuesDict, typeId, devId):
        self.logger.debug(u"validateDeviceConfigUi: typeId = {}, devId = {}".format(typeId, devId))

        errorsDict = indigo.Dict()

        if typeId == DEV_SERIAL_GATEWAY:
            valuesDict['address'] = self.getSerialPortUrl(valuesDict, 'serialPort')


        elif typeId == DEV_KEYPAD and bool(valuesDict[PROP_KEYPADBUT_DISPLAY_LED_STATE]) and int(valuesDict[PROP_KEYPADBUT]) < 80:
            valuesDict[PROP_KEYPADBUT_DISPLAY_LED_STATE] = False
            self.logger.debug(u"validateDeviceConfigUi: forced PROP_KEYPADBUT_DISPLAY_LED_STATE to False for keypad # {}, button # {}".format(
                valuesDict[PROP_INTEGRATION_ID], valuesDict[PROP_KEYPADBUT]))

        if len(errorsDict) > 0:
            return (False, valuesDict, errorsDict)

        return (True, valuesDict)

    def runConcurrentThread(self):

        if self.pluginPrefs.get(u"queryAtStartup", False):
            self.queryAllDevices()

        try:
            while True:

                for gatewayID, gateway in self.gateways.items():
                    if gateway.connected:
                        cmd = gateway.poll
                        if cmd:
                            self._processCommand(cmd.rstrip(), gatewayID)
                    else:
                        gateway.start()

                self.buttonMultiPressCheck()
                self.sleep(0.1)

        except self.StopThread:
            pass

    #########################################
    # Poll registered devices for status
    #########################################

    def queryAllDevices(self):
        for dev in indigo.devices.iter("self"):
            indigo.device.statusRequest(dev)

    def closedPrefsConfigUi(self, valuesDict, userCancelled):
        if userCancelled:
            self.logger.debug(u"closedPrefsConfigUi: User Cancelled")
            return

        logLevel = int(valuesDict.get(u"logLevel", logging.INFO))
        if logLevel != self.logLevel:
            self.logLevel = logLevel
            self.indigo_log_handler.setLevel(self.logLevel)
            self.logger.debug(u"New logLevel = {}".format(self.logLevel))

    ########################################

    def _sendCommand(self, cmd, gateway):
        try:
            self.gateways[int(gateway)].send(cmd)
        except KeyError:
            self.logger.error(u"Unable to send cmd {} to gateway {}.  Possible disabled gateway device.".format(cmd, gateway))

    def _processCommand(self, cmd, gatewayID):
        self.logger.threaddebug(u"Received command: {} from Gateway {}".format(cmd, gatewayID))

        if len(cmd) > 0:
            if "~OUTPUT" in cmd:
                self._cmdOutputChange(cmd, gatewayID)
            elif "~DEVICE" in cmd:
                self._cmdDeviceChange(cmd, gatewayID)
            elif "~HVAC" in cmd:
                self._cmdHvacChange(cmd, gatewayID)
            elif "~GROUP" in cmd:
                self._cmdGroup(cmd, gatewayID)
            elif "~SHADEGRP" in cmd:
                self._cmdShadeGroup(cmd, gatewayID)
            elif "~TIMECLOCK" in cmd:
                self._cmdTimeClock(cmd, gatewayID)
            elif "~MONITORING" in cmd:
                self.logger.debug(u"Main repeater serial interface configured" + cmd)
            elif "~ERROR" in cmd:
                self.logger.debug(u"Gateway {} received: {}".format(gatewayID, cmd))
            elif 'GNET' in cmd:
                # command prompt is ready
                self.logger.threaddebug(u"Command prompt received. Device is ready.")
            elif cmd != "!":
                self.logger.debug(u"Gateway {} Unrecognized command: {}".format(gatewayID, cmd))

    def _cmdOutputChange(self, cmd, gatewayID):
        self.logger.threaddebug(u"Received an Output message: " + cmd)
        cmdArray = cmd.split(',')
        id = "{}:{}".format(gatewayID, cmdArray[1])
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
                    self.logger.debug(u"Received: Switch {} {}".format(switch.name, "turned Off"))
                else:
                    switch.updateStateOnServer(ONOFF, True)
                    self.logger.debug(u"Received: Switch {} {}".format(switch.name, "turned On"))

            elif id in self.ccos:
                cco = self.ccos[id]
                ccoType = cco.pluginProps[PROP_CCO_TYPE]
                if ccoType == "sustained":
                    if int(level) == 0:
                        cco.updateStateOnServer(ONOFF, False)
                    else:
                        cco.updateStateOnServer(ONOFF, True)
                if level == 0.0:
                    self.logger.debug(u"Received: CCO {} {}".format(cco.name, "Opened"))
                else:
                    self.logger.debug(u"Received: CCO {} {}".format(cco.name, "Closed"))

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
            self.logger.warning(u"Received Unknown Action Code: {}".format(cmd))
        return

    def _cmdDeviceChange(self, cmd, gatewayID):
        self.logger.threaddebug(u"Received a Device message: " + cmd)

        cmdArray = cmd.split(',')
        id = cmdArray[1]
        button = cmdArray[2]
        action = cmdArray[3]

        if action == '22':  # battery status report
            self._doBatteryUpdate(gatewayID, cmdArray)
            return

        elif action == '2':  # this is a motion sensor
            if cmdArray[4] == '3':
                status = '1'
            elif cmdArray[4] == '4':
                status = '0'
            else:
                status = None
        elif action == '3':
            status = '1'
        elif action == '4':
            status = '0'
        else:
            status = cmdArray[4]

        keypadid = "{}:{}.{}".format(gatewayID, id, button)

        if keypadid in self.phantomButtons:
            self.logger.debug(u"Received a phantom button status message: " + cmd)
            dev = self.phantomButtons[keypadid]
            if status == '0':
                dev.updateStateOnServer(ONOFF, False)
            elif status == '1':
                dev.updateStateOnServer(ONOFF, True)

        if keypadid in self.keypads:
            self.logger.debug(u"Received a keypad button/LED status message: " + cmd)
            dev = indigo.devices[self.keypads[keypadid]]
            if status == '0':
                dev.updateStateOnServer(ONOFF, False)
            elif status == '1':
                dev.updateStateOnServer(ONOFF, True)

            if dev.pluginProps[PROP_KEYPADBUT_DISPLAY_LED_STATE]:  # Also display this LED state on its corresponding button

                keypadid = "{}:{}.{}".format(gatewayID, id, int(button) - 80)  # Convert LED ID to button ID
                if keypadid in self.keypads:
                    keypad = indigo.devices[self.keypads[keypadid]]
                    self.logger.debug(u"Updating button status with state of LED ({}) for keypadID {}".format(status, keypadid))
                    if status == '0':
                        keypad.updateStateOnServer(ONOFF, False)
                    elif status == '1':
                        keypad.updateStateOnServer(ONOFF, True)
                else:
                    self.logger.error(
                        "WARNING: Invalid ID ({}) specified for LED.   Must be ID of button + 80.  Please correct and reload the plugin.".format(
                            keypadid))
                    self.logger.debug(keypadid)

            if action == '3':  # Check for triggers and linked devices
                self.buttonTriggerCheck(id, button, gatewayID)

        if keypadid in self.picos:
            self.logger.debug(u"Received a pico button status message: " + cmd)
            dev = self.picos[keypadid]
            if status == '0':
                dev.updateStateOnServer(ONOFF, False)
            elif status == '1':
                dev.updateStateOnServer(ONOFF, True)

            if action == '3':  # Check for triggers and linked devices
                self.buttonTriggerCheck(id, button, gatewayID)

        if keypadid in self.ccis:
            self.logger.debug(u"Received a CCI status message: " + cmd)
            dev = self.ccis[keypadid]
            if status == '0':
                dev.updateStateOnServer(ONOFF, False)
                self.logger.info(u"Received: CCI {} {}".format(dev.name, "Opened"))
            elif status == '1':
                dev.updateStateOnServer(ONOFF, True)
                self.logger.info(u"Received: CCI {} {}".format(dev.name, "Closed"))

        sensorid = "{}:{}".format(gatewayID, id)

        if sensorid in self.sensors:
            self.logger.debug(u"Received a sensor status message: " + cmd)
            dev = self.sensors[sensorid]
            if status == '0':
                dev.updateStateOnServer(ONOFF, False)
                self.logger.info(u"Received: Motion Sensor {} {}".format(dev.name, "vacancy detected"))
            elif status == '1':
                dev.updateStateOnServer(ONOFF, True)
                self.logger.info(u"Received: Motion Sensor {} {}".format(dev.name, "motion detected"))

    def _doBatteryUpdate(self, gatewayID, cmdArray):
        id = cmdArray[1]
        button = cmdArray[2]
        battery = (cmdArray[5] == "1")
        batteryLow = (cmdArray[6] == "2")

        self.logger.threaddebug(u"Received a Battery update, IntergrationID = {}, battery = {}, batteryLow = {}".format(id, battery, batteryLow))
        if not battery:  # External power
            return

        device = None
        devAddress = "{}:{}.{}".format(gatewayID, id, button)
        if devAddress in self.picos:
            device = self.picos[devAddress]

        devAddress = "{}:{}".format(gatewayID, id)
        if devAddress in self.sensors:
            device = self.sensors[devAddress]

        devAddress = "{}:{}".format(gatewayID, id)
        if devAddress in self.shades:
            device = self.shades[devAddress]

        if not device:
            return

        if battery and not device.pluginProps.get(PROP_SUPPORTS_BATTERY, False):
            self.update_plugin_property(device, PROP_SUPPORTS_BATTERY, "True")
        if batteryLow:
            device.updateStateOnServer('batteryLevel', 10)
        else:
            device.updateStateOnServer('batteryLevel', 90)

    # IP comm has not yet been tested with _cmdHvacChange().  Currently left as is -vic13
    def _cmdHvacChange(self, cmd, gatewayID):
        self.logger.debug(u"Received an HVAC message: " + cmd)
        cmdArray = cmd.split(',')
        id = "{}:{}".format(gatewayID, cmdArray[1])
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
                mode = cmdArray[3]  # 1=off, 2=heat, 3=cool, 4=auto, 5=em. heat
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

    def _cmdTimeClock(self, cmd, gatewayID):
        self.logger.debug(u"Received a TimeClock message: {}".format(cmd))
        cmdArray = cmd.split(',')
        self.eventTriggerCheck(cmdArray[3], gatewayID)

    def _cmdGroup(self, cmd, gatewayID):
        self.logger.debug(u"Received a Group message:  {}".format(cmd))
        cmdArray = cmd.split(',')
        self.groupTriggerCheck(cmdArray[1], gatewayID, cmdArray[3])

        address = "{}:Group.{}".format(gatewayID, cmdArray[1])
        group = self.groups.get(address, None)
        if not group:
            return
        if cmdArray[3] == "3":
            group.updateStateOnServer(ONOFF, True)
        elif cmdArray[3] == "4":
            group.updateStateOnServer(ONOFF, False)

    def _cmdShadeGroup(self, cmd, gatewayID):
        self.logger.debug(u"Received a Shade Group message:  {}".format(cmd))

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
                sendCmd = ("#DEVICE," + str(int(integration_id)) + "," + str(int(phantom_button) - 100) + ",3,")  # Press button

            elif dev.deviceTypeId == DEV_PICO:
                pico = dev.pluginProps[PROP_INTEGRATION_ID]
                button = dev.pluginProps[PROP_COMPONENT_ID]
                sendCmd = ("#DEVICE," + pico + "," + button + ",3")  # Press button

            elif dev.deviceTypeId == DEV_KEYPAD:
                keypad = dev.pluginProps[PROP_INTEGRATION_ID]
                keypadButton = dev.pluginProps[PROP_COMPONENT_ID]
                if (int(keypadButton) > 80):
                    sendCmd = ("#DEVICE," + keypad + "," + str(int(keypadButton)) + ",9,1")  # Turn on an LED
                else:
                    sendCmd = ("#DEVICE," + keypad + "," + str(int(keypadButton)) + ",3")  # Press button

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
                sendCmd = ("#DEVICE," + cci + "," + str(int(component)) + ",3")

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
                sendCmd = ("#DEVICE," + str(int(integration_id)) + "," + str(int(phantom_button) - 100) + ",4,")  # Release button

            elif dev.deviceTypeId == DEV_PICO:
                pico = dev.pluginProps[PROP_INTEGRATION_ID]
                button = dev.pluginProps[PROP_COMPONENT_ID]
                sendCmd = ("#DEVICE," + pico + "," + button + ",4")  # Release button

            elif dev.deviceTypeId == DEV_KEYPAD:
                keypad = dev.pluginProps[PROP_INTEGRATION_ID]
                keypadButton = dev.pluginProps[PROP_COMPONENT_ID]
                if (int(keypadButton) > 80):
                    sendCmd = ("#DEVICE," + keypad + "," + str(int(keypadButton)) + ",9,0")  # Turn off an LED
                else:
                    sendCmd = ("#DEVICE," + keypad + "," + str(int(keypadButton)) + ",4")  # Release button

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
                sendCmd = ("#DEVICE," + cci + "," + str(int(component)) + ",4")

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
                sendCmd = ("#DEVICE," + str(int(integration_id)) + "," + str(int(phantom_button) - 100) + ",3,")

            elif dev.deviceTypeId == DEV_KEYPAD:
                keypad = dev.pluginProps[PROP_INTEGRATION_ID]
                keypadButton = dev.pluginProps[PROP_COMPONENT_ID]
                if (int(keypadButton) > 80):
                    if dev.onState == True:
                        sendCmd = ("#DEVICE," + keypad + "," + str(int(keypadButton)) + ",9,0")  # Turn off an LED
                    else:
                        sendCmd = ("#DEVICE," + keypad + "," + str(int(keypadButton)) + ",9,1")  # Turn on an LED
                else:
                    if dev.onState == True:
                        sendCmd = ("#DEVICE," + keypad + "," + str(int(keypadButton)) + ",4")  # Release button
                    else:
                        sendCmd = ("#DEVICE," + keypad + "," + str(int(keypadButton)) + ",3")  # Press button

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
                    sendCmd = ("#DEVICE," + cci + "," + str(int(component)) + ",4")
                else:
                    sendCmd = ("#DEVICE," + cci + "," + str(int(component)) + ",3")

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
                sendCmd = ("?DEVICE," + str(int(integration_id)) + "," + str(int(phantom_button)) + ",9,")

            elif dev.deviceTypeId == DEV_KEYPAD:
                integration_id = dev.pluginProps[PROP_INTEGRATION_ID]
                keypadButton = dev.pluginProps[PROP_COMPONENT_ID]
                if (int(keypadButton) > 80):
                    sendCmd = ("?DEVICE," + integration_id + "," + str(int(keypadButton)) + ",9")
                else:
                    sendCmd = ("?DEVICE," + integration_id + "," + str(int(keypadButton) + 80) + ",9")

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
            self.logger.debug(u"{}: actionControlDimmerRelay sending: '{}' to gateway {}".format(dev.name, sendCmd, gateway))
            self._sendCommand(sendCmd, gateway)

    ######################
    # Sensor Action callback
    ######################
    def actionControlSensor(self, action, dev):
        self.logger.debug(u"{}: This device does not respond to Status Requests".format(dev.name))

    ######################
    # Fan Action callback
    ######################
    def actionControlSpeedControl(self, action, dev):
        sendCmd = ""

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
                    self.update_plugin_property(dev, PROP_LASTSPEED, dev.states[ACTUALSPEED])
                dev.updateStateOnServer(ACTUALSPEED, 0)
                dev.updateStateOnServer(ONOFF, False)
                dev.updateStateOnServer(SPEEDINDEX, "0")

        ###### TOGGLE ######
        elif action.speedControlAction == indigo.kSpeedControlAction.Toggle:
            self.logger.debug(u"{}: Toggle".format(dev.name))

            if dev.deviceTypeId == DEV_FAN:

                if int(dev.states[ACTUALSPEED]) > 0:  # turn Off
                    sendCmd = "#OUTPUT,{},1,0".format(dev.pluginProps[PROP_INTEGRATION_ID])
                    if dev.states[ACTUALSPEED] > 0:
                        self.update_plugin_property(dev, PROP_LASTSPEED, dev.states[ACTUALSPEED])
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
                    self.update_plugin_property(dev, PROP_LASTSPEED, "25")
                    dev.updateStateOnServer(ONOFF, True)
                    dev.updateStateOnServer(SPEEDINDEX, 1)
                    dev.updateStateOnServer(ACTUALSPEED, 25)
                elif newSpeedIndex == 2:
                    sendCmd = "#OUTPUT,{},1,75".format(dev.pluginProps[PROP_INTEGRATION_ID])
                    self.update_plugin_property(dev, PROP_LASTSPEED, "75")
                    dev.updateStateOnServer(ONOFF, True)
                    dev.updateStateOnServer(SPEEDINDEX, 2)
                    dev.updateStateOnServer(ACTUALSPEED, 75)
                elif newSpeedIndex == 3:
                    sendCmd = "#OUTPUT,{},1,100".format(dev.pluginProps[PROP_INTEGRATION_ID])
                    self.update_plugin_property(dev, PROP_LASTSPEED, "100")
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
                    self.update_plugin_property(dev, PROP_LASTSPEED, dev.states[ACTUALSPEED])

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
                    self.update_plugin_property(dev, PROP_LASTSPEED, dev.states[ACTUALSPEED])
                    dev.updateStateOnServer(ONOFF, False)
                    dev.updateStateOnServer(SPEEDINDEX, 0)
                elif newSpeedIndex == 1:
                    sendCmd = "#OUTPUT,{},1,25".format(dev.pluginProps[PROP_INTEGRATION_ID])
                    self.update_plugin_property(dev, PROP_LASTSPEED, "25")
                    dev.updateStateOnServer(ONOFF, True)
                    dev.updateStateOnServer(SPEEDINDEX, 1)
                    dev.updateStateOnServer(ACTUALSPEED, 25)
                elif newSpeedIndex == 2:
                    sendCmd = "#OUTPUT,{},1,75".format(dev.pluginProps[PROP_INTEGRATION_ID])
                    self.update_plugin_property(dev, PROP_LASTSPEED, "75")
                    dev.updateStateOnServer(ONOFF, True)
                    dev.updateStateOnServer(SPEEDINDEX, 2)
                    dev.updateStateOnServer(ACTUALSPEED, 75)
                elif newSpeedIndex == 3:
                    sendCmd = "#OUTPUT,{},1,100".format(dev.pluginProps[PROP_INTEGRATION_ID])
                    self.update_plugin_property(dev, PROP_LASTSPEED, "100")
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
                    self.update_plugin_property(dev, PROP_LASTSPEED, dev.states[ACTUALSPEED])
                    dev.updateStateOnServer(ONOFF, False)
                    dev.updateStateOnServer(SPEEDINDEX, 0)
                elif newSpeedIndex == 1:
                    sendCmd = "#OUTPUT,{},1,25".format(dev.pluginProps[PROP_INTEGRATION_ID])
                    self.update_plugin_property(dev, PROP_LASTSPEED, "25")
                    dev.updateStateOnServer(ONOFF, True)
                    dev.updateStateOnServer(SPEEDINDEX, 1)
                    dev.updateStateOnServer(ACTUALSPEED, 25)
                elif newSpeedIndex == 2:
                    sendCmd = "#OUTPUT,{},1,75".format(dev.pluginProps[PROP_INTEGRATION_ID])
                    self.update_plugin_property(dev, PROP_LASTSPEED, "75")
                    dev.updateStateOnServer(ONOFF, True)
                    dev.updateStateOnServer(SPEEDINDEX, 2)
                    dev.updateStateOnServer(ACTUALSPEED, 75)
                elif newSpeedIndex == 3:
                    sendCmd = "#OUTPUT,{},1,100".format(dev.pluginProps[PROP_INTEGRATION_ID])
                    self.update_plugin_property(dev, PROP_LASTSPEED, "100")
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
            sendCmd = "#HVAC," + integration_id + ",2," + str(newHeatSetpoint) + "," + str(newCoolSetpoint) + "\r"

        elif action.thermostatAction == indigo.kThermostatAction.SetCoolSetpoint:
            newCoolSetpoint = action.actionValue
            dev.updateStateOnServer("setpointCool", newCoolSetpoint)
            newHeatSetpoint = float(currentHeatSetpoint)
            sendCmd = "#HVAC," + integration_id + ",2," + str(newHeatSetpoint) + "," + str(newCoolSetpoint) + "\r"

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

            sendCmd = "?HVAC," + integration_id + ",1,"  # get temperature
            self._sendCommand(sendCmd, gateway)
            self.logger.debug(u"{}: actionControlSpeedControl sent: '{}' to gateway {}".format(dev.name, sendCmd, gateway))

            sendCmd = "?HVAC," + integration_id + ",2,"  # get heat and cool setpoints
            self._sendCommand(sendCmd, gateway)
            self.logger.debug(u"{}: actionControlSpeedControl sent: '{}' to gateway {}".format(dev.name, sendCmd, gateway))

            sendCmd = "?HVAC," + integration_id + ",3,"  # get operating mode
            self._sendCommand(sendCmd, gateway)
            self.logger.debug(u"{}: actionControlSpeedControl sent: '{}' to gateway {}".format(dev.name, sendCmd, gateway))

            sendCmd = "?HVAC," + integration_id + ",4,"  # get fan mode
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
    ########################################

    def setFanSpeed(self, pluginAction, dev):

        gateway = dev.pluginProps[PROP_GATEWAY]
        integrationID = dev.pluginProps[PROP_INTEGRATION_ID]

        fanSpeed = pluginAction.props["fanSpeed"]
        sendCmd = "#OUTPUT,{},1,{}".format(integrationID, fanSpeed)
        self._sendCommand(sendCmd, gateway)
        self.logger.debug(u"{}: Set fan speed {} to {}".format(dev.name, fanSpeed, gateway))

    def fadeDimmer(self, pluginAction, dev):

        gateway = dev.pluginProps[PROP_GATEWAY]
        integrationID = dev.pluginProps[PROP_INTEGRATION_ID]

        brightness = indigo.activePlugin.substitute(pluginAction.props["brightness"])
        fadeTime = indigo.activePlugin.substitute(pluginAction.props["fadeTime"])

        m, s = divmod(int(fadeTime), 60)
        sendCmd = ("#OUTPUT,{},1,{},{:02}:{:02}".format(integrationID, brightness, m, s))
        self._sendCommand(sendCmd, gateway)
        self.logger.info(u"{}: Set brightness to {} with fade {}".format(dev.name, brightness, fadeTime))

    def startRaising(self, pluginAction, dev):

        gateway = dev.pluginProps[PROP_GATEWAY]
        integrationID = dev.pluginProps[PROP_INTEGRATION_ID]

        sendCmd = ("#OUTPUT," + integrationID + ",2")
        self._sendCommand(sendCmd, gateway)
        self.logger.info(u"{}: Start Raising".format(dev.name))

    def startLowering(self, pluginAction, dev):

        gateway = dev.pluginProps[PROP_GATEWAY]
        integrationID = dev.pluginProps[PROP_INTEGRATION_ID]

        sendCmd = ("#OUTPUT," + integrationID + ",3")
        self._sendCommand(sendCmd, gateway)
        self.logger.info(u"{}: Start Lowering".format(dev.name))

    def stopRaiseLower(self, pluginAction, dev):

        gateway = dev.pluginProps[PROP_GATEWAY]
        integrationID = dev.pluginProps[PROP_INTEGRATION_ID]

        sendCmd = ("#OUTPUT," + integrationID + ",4")
        self._sendCommand(sendCmd, gateway)
        self.logger.info(u"{}: Stop Raising/Lowering".format(dev.name))

    def sendRawCommand(self, pluginAction):

        gateway = pluginAction.props[PROP_GATEWAY]
        sendCmd = indigo.activePlugin.substitute(pluginAction.props["commandString"])

        self._sendCommand(sendCmd, gateway)
        self.logger.debug(u"Sent Raw Command: '{}' to gateway {}".format(sendCmd, gateway))

    def getBatteryLevels(self, pluginAction):

        gateway = pluginAction.props[PROP_GATEWAY]
        self.logger.debug(u"Getting Battery Status for gateway {}".format(gateway))

        for sensorid in self.sensors:
            device = self.sensors[sensorid]
            integrationID = device.pluginProps[PROP_INTEGRATION_ID]
            sendCmd = ("?DEVICE,{},1,22".format(integrationID))
            self._sendCommand(sendCmd, gateway)

        for shadeid in self.shades:
            device = self.shades[shadeid]
            integrationID = device.pluginProps[PROP_INTEGRATION_ID]
            sendCmd = ("?DEVICE,{},1,22".format(integrationID))
            self._sendCommand(sendCmd, gateway)

        for buttonid in self.picos:
            device = self.picos[buttonid]
            integrationID = device.pluginProps[PROP_INTEGRATION_ID]
            componentID = device.pluginProps[PROP_COMPONENT_ID]
            sendCmd = ("?DEVICE,{},{},22".format(integrationID, componentID))
            self._sendCommand(sendCmd, gateway)

    ########################################

    def get_gateway_list(self, filter="", valuesDict=None, typeId="", targetId=0):
        self.logger.threaddebug(u"get_gateway_list: typeId = {}, targetId = {}, valuesDict = {}".format(typeId, targetId, valuesDict))
        gateways = [
            (gateway.dev.id, indigo.devices[gateway.dev.id].name)
            for gateway in self.gateways.values()
        ]
        return gateways

    def roomListGenerator(self, filter=None, valuesDict=None, typeId=0, targetId=0):
        self.logger.threaddebug(u"roomListGenerator, typeId = {}, targetId = {}, valuesDict = {}".format(typeId, targetId, valuesDict))
        retList = []
        for room in self.roomList:
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

        for buttonId, devID in self.keypads.items():
            buttonDev = indigo.devices[devID]
            if buttonDev.pluginProps.get(PROP_ISBUTTON, None) and (buttonDev.pluginProps.get(PROP_ROOM, None) == room):
                self.logger.threaddebug(u"pickKeypadButton adding: {} ({})".format(buttonId, buttonDev.name))
                retList.append((buttonId, buttonDev.name))

        retList.sort(key=lambda tup: tup[1])
        return retList

    def pickEvent(self, filter=None, valuesDict=None, typeId=0, targetId=0):
        self.logger.threaddebug(u"pickEvent, typeId = {}, targetId = {}, valuesDict = {}".format(typeId, targetId, valuesDict))
        retList = []
        for dev in indigo.devices.iter("self.ra2TimeClockEvent"):
            if dev.pluginProps[PROP_GATEWAY] == valuesDict[PROP_GATEWAY]:
                event = dev.pluginProps[PROP_EVENT]
                self.logger.threaddebug(u"pickEvent adding: {}".format(event))
                retList.append((event, dev.name))
        retList.sort(key=lambda tup: tup[1])
        return retList

    def pickGroup(self, filter=None, valuesDict=None, typeId=0, targetId=0):
        self.logger.threaddebug(u"pickGroup, typeId = {}, targetId = {}, valuesDict = {}".format(typeId, targetId, valuesDict))
        retList = []
        for dev in indigo.devices.iter("self.ra2Group"):
            if dev.pluginProps[PROP_GATEWAY] == valuesDict[PROP_GATEWAY]:
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
                if dev.pluginId != self.pluginId:
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
        deviceID = parts[0]
        componentID = parts[1]
        buttonLEDAddress = "{}.{}".format(deviceID, int(componentID) + 80)
        try:
            buttonLEDDeviceId = unicode(self.keypads[buttonLEDAddress].id)
        except:
            buttonLEDDeviceId = "0"
        linkID = "{}-{}".format(buttonDeviceId, controlledDeviceId)
        if len(linkName) == 0:
            linkName = linkID
        linkItem = {"name": linkName, "buttonDevice": buttonDeviceId, "buttonLEDDevice": buttonLEDDeviceId, "controlledDevice": controlledDeviceId,
                    "buttonAddress": buttonAddress}
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
        for linkID, linkItem in self.linkedDeviceList.items():
            returnList.append((linkID, linkItem["name"]))
        return sorted(returnList, key=lambda item: item[1])

    ########################################

    def logLinkedDevices(self):
        if len(self.linkedDeviceList) == 0:
            self.logger.info(u"No linked Devices")
            return

        fstring = u"{:^25} {:^25} {:^20} {:^20} {:^20} {:^20}"
        self.logger.info(fstring.format("Link ID", "Link Name", "buttonDevice", "buttonLEDDevice", "controlledDevice", "buttonAddress"))
        for linkID, linkItem in self.linkedDeviceList.items():
            self.logger.info(
                fstring.format(linkID, linkItem["name"], linkItem["buttonDevice"], linkItem["buttonLEDDevice"], linkItem["controlledDevice"],
                               linkItem["buttonAddress"]))

    ########################################

    def createCasetaDevicesMenu(self, valuesDict, typeId):

        deviceThread = threading.Thread(target=self.createCasetaDevices, args=(valuesDict,))
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
            self.logger.info(u"Caseta Device '{}' ({})".format(device["Name"], device["ID"]))

            if device["ID"] == 1 and not self.create_bridge_buttons:
                self.logger.debug(u"Skipping Smart Bridge button creation")
                continue

            try:
                areaName = device["Area"]["Name"]
            except:
                areaName = u"Bridge"
            self.logger.debug(u"Using area name '{}'".format(areaName))

            try:
                for button in device["Buttons"]:
                    address = "{}:{}.{}".format(gatewayID, device["ID"], button["Number"])
                    name = u"{} - {} ({})".format(areaName, button.get("Name", device["Name"]), address)

                    props = {
                        PROP_ROOM: areaName,
                        PROP_LIST_TYPE: "button",
                        PROP_GATEWAY: gatewayID,
                        PROP_INTEGRATION_ID: str(device["ID"]),
                        PROP_COMPONENT_ID: str(button["Number"]),
                        PROP_BUTTONTYPE: "Unknown",
                        PROP_ISBUTTON: "True"
                    }
                    self.createLutronDevice(DEV_PICO, name, address, props, areaName)
            except:
                pass

        for zone in casetaData["LIPIdList"]["Zones"]:
            self.logger.info(u"Caseta Zone '{}' ({}), Area = {}".format(zone["Name"], zone["ID"], zone["Area"]["Name"]))

            try:
                areaName = zone["Area"]["Name"]
            except:
                areaName = u"Unknown"

            address = "{}:{}".format(gatewayID, zone["ID"])
            name = u"{} - {} ({})".format(areaName, zone["Name"], zone["ID"])
            props = {
                PROP_ROOM: areaName,
                PROP_GATEWAY: gatewayID,
                PROP_INTEGRATION_ID: str(zone["ID"]),
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
        deviceThread = threading.Thread(target=self.createRRA2Devices, args=(valuesDict,))
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
            self.logger.info(
                u"Creating Devices from file: %s, Grouping = %s, Create unprogrammed keypad buttons = %s, Create unprogrammed phantom buttons = %s" % \
                (valuesDict["xmlFileName"], self.group_by, self.create_unused_keypad, self.create_unused_phantom))

            xmlFile = os.path.expanduser(valuesDict["xmlFileName"])
            try:
                root = ET.parse(xmlFile).getroot()
            except:
                self.logger.error(u"Unable to parse XML file: {}".format(xmlFile))
                self.threadLock.release()
                return

            self.logger.info(u"Creating Devices file read completed, parsing data...")

        else:

            self.logger.info(
                u"Creating RRA2 Devices from gateway {}, Grouping = {}, Create unprogrammed keypad buttons = {}, Create unprogrammed phantom buttons = {}".format(
                    gatewayID, self.group_by, self.create_unused_keypad, self.create_unused_phantom))
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
            self.logger.info("Finding devices in '{}'".format(room.attrib['Name']))

            for device in room.findall('DeviceGroups/Device'):
                self.logger.debug("Device: {} ({},{})".format(device.attrib['Name'], device.attrib['IntegrationID'], device.attrib['DeviceType']))
                if device.attrib['DeviceType'] == "MAIN_REPEATER":
                    for component in device.findall('Components/Component'):
                        self.logger.debug("Component: {} ({})".format(component.attrib['ComponentNumber'], component.attrib['ComponentType']))
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
                                PROP_ROOM: room.attrib['Name'],
                                PROP_GATEWAY: gatewayID,
                                PROP_INTEGRATION_ID: device.attrib['IntegrationID'],
                                PROP_COMPONENT_ID: button,
                                PROP_BUTTONTYPE: buttonType,
                                PROP_ISBUTTON: "True"
                            }
                            self.createLutronDevice(DEV_PHANTOM_BUTTON, name, address, props, room.attrib['Name'])

                        elif component.attrib['ComponentType'] == "LED":  # ignore LEDs for phantom buttons
                            pass

                        else:
                            self.logger.error("Unknown Component Type: {} ({})".format(component.attrib['Name'], component.attrib['ComponentType']))

                else:
                    self.logger.error("Unknown Device Type: {} ({})".format(device.attrib['Name'], device.attrib['DeviceType']))

            for output in room.findall('Outputs/Output'):
                self.logger.debug("Output: {} ({}) {}".format(output.attrib['Name'], output.attrib['IntegrationID'], output.attrib['OutputType']))

                if output.attrib['OutputType'] in ["INC", "MLV", "ELV", "AUTO_DETECT"]:
                    address = gatewayID + ":" + output.attrib['IntegrationID']
                    name = u"{} - Dimmer {} - {}".format(room.attrib['Name'], output.attrib['IntegrationID'], output.attrib['Name'])
                    props = {
                        PROP_ROOM: room.attrib['Name'],
                        PROP_GATEWAY: gatewayID,
                        PROP_INTEGRATION_ID: output.attrib['IntegrationID'],
                        PROP_OUTPUTTYPE: output.attrib[PROP_OUTPUTTYPE],
                    }
                    self.createLutronDevice(DEV_DIMMER, name, address, props, room.attrib['Name'])

                elif output.attrib['OutputType'] == "NON_DIM":
                    address = gatewayID + ":" + output.attrib['IntegrationID']
                    name = u"{} - Switch {} - {}".format(room.attrib['Name'], output.attrib['IntegrationID'], output.attrib['Name'])
                    props = {
                        PROP_ROOM: room.attrib['Name'],
                        PROP_GATEWAY: gatewayID,
                        PROP_INTEGRATION_ID: output.attrib['IntegrationID'],
                        PROP_OUTPUTTYPE: output.attrib[PROP_OUTPUTTYPE]
                    }
                    self.createLutronDevice(DEV_SWITCH, name, address, props, room.attrib['Name'])

                elif output.attrib['OutputType'] == "SYSTEM_SHADE":
                    address = gatewayID + ":" + output.attrib['IntegrationID']
                    name = u"{} - Shade {} - {}".format(room.attrib['Name'], output.attrib['IntegrationID'], output.attrib['Name'])
                    props = {
                        PROP_ROOM: room.attrib['Name'],
                        PROP_GATEWAY: gatewayID,
                        PROP_INTEGRATION_ID: output.attrib['IntegrationID'],
                        PROP_OUTPUTTYPE: output.attrib[PROP_OUTPUTTYPE]
                    }
                    self.createLutronDevice(DEV_SHADE, name, address, props, room.attrib['Name'])

                elif output.attrib['OutputType'] == "CEILING_FAN_TYPE":
                    address = gatewayID + ":" + output.attrib['IntegrationID']
                    name = u"{} - Fan {} - {}".format(room.attrib['Name'], output.attrib['IntegrationID'], output.attrib['Name'])
                    props = {
                        PROP_ROOM: room.attrib['Name'],
                        PROP_GATEWAY: gatewayID,
                        PROP_INTEGRATION_ID: output.attrib['IntegrationID'],
                        PROP_OUTPUTTYPE: output.attrib[PROP_OUTPUTTYPE]
                    }
                    self.createLutronDevice(DEV_FAN, name, address, props, room.attrib['Name'])

                elif output.attrib['OutputType'] == "CCO_PULSED":
                    address = gatewayID + ":" + output.attrib['IntegrationID']
                    name = u"{} - VCRX CCO Momentary {} - {}".format(room.attrib['Name'], output.attrib['IntegrationID'], output.attrib['Name'])
                    props = {
                        PROP_ROOM: room.attrib['Name'],
                        PROP_GATEWAY: gatewayID,
                        PROP_INTEGRATION_ID: output.attrib['IntegrationID'],
                        PROP_CCO_TYPE: "momentary",
                        PROP_SUPPORTS_STATUS_REQUEST: "False",
                        PROP_OUTPUTTYPE: output.attrib[PROP_OUTPUTTYPE]
                    }
                    self.createLutronDevice(DEV_CCO, name, address, props, room.attrib['Name'])

                elif output.attrib['OutputType'] == "CCO_MAINTAINED":
                    address = gatewayID + ":" + output.attrib['IntegrationID']
                    name = u"{} - VCRX CCO Sustained {} - {}".format(room.attrib['Name'], output.attrib['IntegrationID'], output.attrib['Name'])
                    props = {
                        PROP_ROOM: room.attrib['Name'],
                        PROP_GATEWAY: gatewayID,
                        PROP_INTEGRATION_ID: output.attrib['IntegrationID'],
                        PROP_CCO_TYPE: "sustained",
                        PROP_SUPPORTS_STATUS_REQUEST: "True",
                        PROP_OUTPUTTYPE: output.attrib[PROP_OUTPUTTYPE]
                    }
                    self.createLutronDevice(DEV_CCO, name, address, props, room.attrib['Name'])

                elif output.attrib['OutputType'] == "HVAC":
                    pass

                else:
                    self.logger.error(
                        "Unknown Output Type: {} ({}, {})".format(output.attrib['Name'], output.attrib['OutputType'], output.attrib['IntegrationID']))

            for device in room.findall('DeviceGroups/DeviceGroup/Devices/Device'):
                self.logger.debug("Device: {} ({},{})".format(device.attrib['Name'], device.attrib['IntegrationID'], device.attrib['DeviceType']))

                if device.attrib['DeviceType'] == "SEETOUCH_KEYPAD" or device.attrib['DeviceType'] == "HYBRID_SEETOUCH_KEYPAD" or device.attrib[
                    'DeviceType'] == "SEETOUCH_TABLETOP_KEYPAD":
                    for component in device.findall('Components/Component'):
                        self.logger.debug("Component: {} ({})".format(component.attrib['ComponentNumber'], component.attrib['ComponentType']))
                        if component.attrib['ComponentType'] == "BUTTON":

                            assignments = len(component.findall('Button/Actions/Action/Presets/Preset/PresetAssignments/PresetAssignment'))
                            if not self.create_unused_keypad and assignments == 0:
                                continue

                            address = gatewayID + ":" + device.attrib['IntegrationID'] + "." + component.attrib['ComponentNumber']
                            keypadType = device.attrib['DeviceType']
                            buttonNum = int(component.attrib['ComponentNumber'])
                            if ((keypadType == "SEETOUCH_KEYPAD") or (keypadType == "HYBRID_SEETOUCH_KEYPAD")) and (buttonNum == 16):
                                name = u"{} - {} - Button {:03}.{:02} - Top Lower".format(room.attrib['Name'], device.attrib['Name'],
                                                                                          int(device.attrib['IntegrationID']),
                                                                                          int(component.attrib['ComponentNumber']))
                            elif ((keypadType == "SEETOUCH_KEYPAD") or (keypadType == "HYBRID_SEETOUCH_KEYPAD")) and (buttonNum == 17):
                                name = u"{} - {} - Button {:03}.{:02} - Top Raise".format(room.attrib['Name'], device.attrib['Name'],
                                                                                          int(device.attrib['IntegrationID']),
                                                                                          int(component.attrib['ComponentNumber']))
                            elif ((keypadType == "SEETOUCH_KEYPAD") or (keypadType == "HYBRID_SEETOUCH_KEYPAD")) and (buttonNum == 18):
                                name = u"{} - {} - Button {:03}.{:02} - Bottom Lower".format(room.attrib['Name'], device.attrib['Name'],
                                                                                             int(device.attrib['IntegrationID']),
                                                                                             int(component.attrib['ComponentNumber']))
                            elif ((keypadType == "SEETOUCH_KEYPAD") or (keypadType == "HYBRID_SEETOUCH_KEYPAD")) and (buttonNum == 19):
                                name = u"{} - {} - Button {:03}.{:02} - Bottom Raise".format(room.attrib['Name'], device.attrib['Name'],
                                                                                             int(device.attrib['IntegrationID']),
                                                                                             int(component.attrib['ComponentNumber']))
                            elif (keypadType == "SEETOUCH_TABLETOP_KEYPAD") and (buttonNum == 20):
                                name = u"{} - {} - Button {:03}.{:02}} - Column 1 Lower".format(room.attrib['Name'], device.attrib['Name'],
                                                                                                int(device.attrib['IntegrationID']),
                                                                                                int(component.attrib['ComponentNumber']))
                            elif (keypadType == "SEETOUCH_TABLETOP_KEYPAD") and (buttonNum == 21):
                                name = u"{} - {} - Button {:03}.{:02} - Column 1 Raise".format(room.attrib['Name'], device.attrib['Name'],
                                                                                               int(device.attrib['IntegrationID']),
                                                                                               int(component.attrib['ComponentNumber']))
                            elif (keypadType == "SEETOUCH_TABLETOP_KEYPAD") and (buttonNum == 22):
                                name = u"{} - {} - Button {:03}.{:02}} - Column 2 Lower".format(room.attrib['Name'], device.attrib['Name'],
                                                                                                int(device.attrib['IntegrationID']),
                                                                                                int(component.attrib['ComponentNumber']))
                            elif (keypadType == "SEETOUCH_TABLETOP_KEYPAD") and (buttonNum == 23):
                                name = u"{} - {} - Button {:03}.{:02} - Column 2 Raise".format(room.attrib['Name'], device.attrib['Name'],
                                                                                               int(device.attrib['IntegrationID']),
                                                                                               int(component.attrib['ComponentNumber']))
                            elif (keypadType == "SEETOUCH_TABLETOP_KEYPAD") and (buttonNum == 24):
                                name = u"{} - {} - Button {:03}.{:02} - Column 3 Lower".format(room.attrib['Name'], device.attrib['Name'],
                                                                                               int(device.attrib['IntegrationID']),
                                                                                               int(component.attrib['ComponentNumber']))
                            elif (keypadType == "SEETOUCH_TABLETOP_KEYPAD") and (buttonNum == 25):
                                name = u"{} - {} - Button {:03}.{:02} - Column 3 Raise".format(room.attrib['Name'], device.attrib['Name'],
                                                                                               int(device.attrib['IntegrationID']),
                                                                                               int(component.attrib['ComponentNumber']))
                            else:
                                name = u"{} - {} - Button {:03}.{:02}".format(room.attrib['Name'], device.attrib['Name'],
                                                                              int(device.attrib['IntegrationID']),
                                                                              int(component.attrib['ComponentNumber']))
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
                                PROP_ROOM: room.attrib['Name'],
                                PROP_LIST_TYPE: "button",
                                PROP_GATEWAY: gatewayID,
                                PROP_INTEGRATION_ID: device.attrib['IntegrationID'],
                                PROP_COMPONENT_ID: component.attrib['ComponentNumber'],
                                PROP_KEYPADBUT_DISPLAY_LED_STATE: "false",
                                PROP_BUTTONTYPE: buttonType,
                                PROP_ISBUTTON: "True"
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
                                PROP_ROOM: room.attrib['Name'],
                                PROP_LIST_TYPE: "LED",
                                PROP_GATEWAY: gatewayID,
                                PROP_INTEGRATION_ID: device.attrib['IntegrationID'],
                                PROP_COMPONENT_ID: keypadLED,
                                PROP_KEYPADBUT_DISPLAY_LED_STATE: "False"
                            }
                            self.createLutronDevice(DEV_KEYPAD, name, address, props, room.attrib['Name'])

                        elif component.attrib['ComponentType'] == "LED":
                            pass  # LED device created same time as button

                        else:
                            self.logger.error("Unknown Component Type: {} ({})".format(component.attrib['Name'], component.attrib['ComponentType']))

                elif device.attrib['DeviceType'] == "VISOR_CONTROL_RECEIVER":
                    for component in device.findall('Components/Component'):
                        self.logger.debug("Component: {} ({})".format(component.attrib['ComponentNumber'], component.attrib['ComponentType']))
                        if component.attrib['ComponentType'] == "BUTTON":
                            assignments = len(component.findall('Button/Actions/Action/Presets/Preset/PresetAssignments/PresetAssignment'))
                            if not self.create_unused_keypad and assignments == 0:
                                continue

                            name = u"{} - VCRX Button {:03}.{:02}".format(room.attrib['Name'], int(device.attrib['IntegrationID']),
                                                                          int(component.attrib['ComponentNumber']))
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
                                PROP_ROOM: room.attrib['Name'],
                                PROP_LIST_TYPE: "button",
                                PROP_GATEWAY: gatewayID,
                                PROP_INTEGRATION_ID: device.attrib['IntegrationID'],
                                PROP_COMPONENT_ID: component.attrib['ComponentNumber'],
                                PROP_KEYPADBUT_DISPLAY_LED_STATE: "false",
                                PROP_BUTTONTYPE: buttonType,
                                PROP_ISBUTTON: "True"
                            }
                            self.createLutronDevice(DEV_KEYPAD, name, address, props, room.attrib['Name'])

                            # create button LED, if needed for the button

                            name = name + " LED"
                            keypadLED = str(int(component.attrib['ComponentNumber']) + 80)
                            address = gatewayID + ":" + device.attrib['IntegrationID'] + "." + keypadLED
                            props = {
                                PROP_ROOM: room.attrib['Name'],
                                PROP_LIST_TYPE: "LED",
                                PROP_GATEWAY: gatewayID,
                                PROP_INTEGRATION_ID: device.attrib['IntegrationID'],
                                PROP_COMPONENT_ID: keypadLED,
                                PROP_KEYPADBUT_DISPLAY_LED_STATE: "False"
                            }
                            self.createLutronDevice(DEV_KEYPAD, name, address, props, room.attrib['Name'])

                        elif component.attrib['ComponentType'] == "LED":
                            pass

                        elif component.attrib['ComponentType'] == "CCI":
                            name = u"{} - VCRX CCI Input {:03}.{:02}".format(room.attrib['Name'], int(device.attrib['IntegrationID']),
                                                                             int(component.attrib['ComponentNumber']))
                            address = gatewayID + ":" + device.attrib['IntegrationID'] + "." + component.attrib['ComponentNumber']
                            props = {
                                PROP_ROOM: room.attrib['Name'],
                                PROP_GATEWAY: gatewayID,
                                PROP_INTEGRATION_ID: device.attrib['IntegrationID'],
                                PROP_COMPONENT_ID: component.attrib['ComponentNumber'],
                                PROP_SUPPORTS_STATUS_REQUEST: "False"
                            }
                            self.createLutronDevice(DEV_CCI, name, address, props, room.attrib['Name'])

                        else:
                            self.logger.error("Unknown Component Type: {} ({})".format(component.attrib['Name'], component.attrib['ComponentType']))

                elif device.attrib['DeviceType'] == "PICO_KEYPAD":
                    for component in device.findall('Components/Component'):
                        self.logger.debug("Component: {} ({})".format(component.attrib['ComponentNumber'], component.attrib['ComponentType']))
                        if component.attrib['ComponentType'] == "BUTTON":

                            assignments = len(component.findall('Button/Actions/Action/Presets/Preset/PresetAssignments/PresetAssignment'))
                            if not self.create_unused_keypad and assignments == 0:
                                continue

                            name = u"{} - {} - Button {:03}.{:02}".format(room.attrib['Name'], device.attrib['Name'],
                                                                          int(device.attrib['IntegrationID']),
                                                                          int(component.attrib['ComponentNumber']))
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
                                PROP_ROOM: room.attrib['Name'],
                                PROP_GATEWAY: gatewayID,
                                PROP_INTEGRATION_ID: device.attrib['IntegrationID'],
                                PROP_COMPONENT_ID: component.attrib['ComponentNumber'],
                                PROP_BUTTONTYPE: buttonType,
                                PROP_ISBUTTON: "True"
                            }
                            self.createLutronDevice(DEV_PICO, name, address, props, room.attrib['Name'])

                        else:
                            self.logger.error("Unknown Component Type: {} ({})".format(component.attrib['Name'], component.attrib['ComponentType']))

                elif device.attrib['DeviceType'] == "MOTION_SENSOR":
                    name = u"{} - Motion Sensor {}".format(room.attrib['Name'], device.attrib['IntegrationID'])
                    address = gatewayID + ":" + device.attrib['IntegrationID']
                    props = {
                        PROP_ROOM: room.attrib['Name'],
                        PROP_GATEWAY: gatewayID,
                        PROP_INTEGRATION_ID: device.attrib['IntegrationID'],
                        PROP_SUPPORTS_STATUS_REQUEST: "False"
                    }
                    self.createLutronDevice(DEV_SENSOR, name, address, props, room.attrib['Name'])

                    # Create a Group (Room) device for every room that has a motion sensors

                    name = u"Room Group {:03} - {}".format(int(room.attrib['IntegrationID']), room.attrib['Name'])
                    address = gatewayID + ":Group." + room.attrib['IntegrationID']
                    props = {
                        PROP_GATEWAY: gatewayID,
                        'group': room.attrib['IntegrationID']
                    }
                    if not address in self.groups:
                        self.createLutronDevice(DEV_GROUP, name, address, props, room.attrib['Name'])

                    if self.create_group_triggers:
                        self.logger.debug("Creating Room Group triggers for: {} ({})".format(name, address))

                        if "Lutron" in indigo.triggers.folders:
                            theFolder = indigo.triggers.folders["Lutron"].id
                        else:
                            self.logger.debug("Creating Trigger Folder: '{}'".format("Lutron"))
                            theFolder = indigo.triggers.folder.create("Lutron").id

                        trigger_exists = False
                        trigger = None
                        for triggerId, trigger in self.groupTriggers.items():
                            if (trigger.pluginProps[PROP_GROUP] == address) and (trigger.pluginProps["occupancyPopUp"] == "3"):
                                trigger_exists = True
                                break

                        if trigger_exists:
                            self.logger.debug("Skipping existing room group trigger: {}, {}".format(trigger.pluginProps[PROP_GROUP],
                                                                                                    trigger.pluginProps["occupancyPopUp"]))

                        else:
                            triggerName = u"{} Occupied".format(name)
                            self.logger.info("Creating Room Group Event trigger: '{}' ({})".format(triggerName, address))
                            indigo.pluginEvent.create(name=triggerName,
                                                      description="",
                                                      folder=theFolder,
                                                      pluginId=self.pluginId,
                                                      pluginTypeId="groupEvent",
                                                      props={PROP_GROUP: address, "occupancyPopUp": "3"}
                                                      )

                        trigger_exists = False
                        for triggerId, trigger in self.groupTriggers.items():
                            if (trigger.pluginProps[PROP_GROUP] == address) and (trigger.pluginProps["occupancyPopUp"] == "4"):
                                trigger_exists = True
                                break

                        if trigger_exists:
                            self.logger.debug("Skipping existing room group trigger: {}, {}".format(trigger.pluginProps[PROP_GROUP],
                                                                                                    trigger.pluginProps["occupancyPopUp"]))

                        else:
                            triggerName = u"{} Unoccupied".format(name)
                            self.logger.info("Creating Room Group Event trigger: '{}' ({})".format(triggerName, address))
                            indigo.pluginEvent.create(name=triggerName,
                                                      description="",
                                                      folder=theFolder,
                                                      pluginId=self.pluginId,
                                                      pluginTypeId="groupEvent",
                                                      props={PROP_GROUP: address, "occupancyPopUp": "4"}
                                                      )

                elif device.attrib['DeviceType'] == "TEMPERATURE_SENSOR":
                    pass

                else:
                    self.logger.error("Unknown Device Type: {} ({})".format(device.attrib['Name'], device.attrib['DeviceType']))

        self.logger.info("Finding Timeclock events...")
        for event in root.iter('TimeClockEvent'):
            self.logger.debug("TimeClockEvent: {} ({})".format(event.attrib['Name'], event.attrib['EventNumber']))
            name = u"Event {:02} - {}".format(int(event.attrib['EventNumber']), event.attrib['Name'])
            address = gatewayID + ":" + "Event.{}".format(event.attrib['EventNumber'])
            props = {
                PROP_GATEWAY: gatewayID,
                PROP_EVENT: event.attrib['EventNumber']
            }
            self.createLutronDevice(DEV_TIMECLOCKEVENT, name, address, props, "TimeClock")

            if self.create_event_triggers:
                self.logger.debug("Creating Event triggers for: {} ({})".format(name, address))

                if "Lutron" in indigo.triggers.folders:
                    theFolder = indigo.triggers.folders["Lutron"].id
                else:
                    self.logger.debug("Creating Trigger Folder: '{}'".format("Lutron"))
                    theFolder = indigo.triggers.folder.create("Lutron").id

                trigger_exists = False
                trigger = None
                for triggerId, trigger in self.eventTriggers.items():
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
                                              pluginId=self.pluginId,
                                              pluginTypeId="timeClockEvent",
                                              props={PROP_EVENT: event.attrib['EventNumber']}
                                              )

        self.logger.info("Finding HVAC devices...")
        for hvac in root.iter('HVAC'):
            self.logger.debug("HVAC: {} ({})".format(hvac.attrib['Name'], hvac.attrib['IntegrationID']))
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

        self.logger.threaddebug(
            "createLutronDevice: devType = {}, name = {}, address = {}, props = {}, room = {}".format(devType, name, address, props, room))

        folderNameDict = {
            DEV_PHANTOM_BUTTON: "Lutron Phantom Buttons",
            DEV_DIMMER: "Lutron Dimmers",
            DEV_SWITCH: "Lutron Switches",
            DEV_KEYPAD: "Lutron Keypads",
            DEV_FAN: "Lutron Fans",
            DEV_SENSOR: "Lutron Sensors",
            DEV_THERMO: "Lutron Thermostats",
            DEV_CCO: "Lutron Switches",
            DEV_CCI: "Lutron Sensors",
            DEV_SHADE: "Lutron Shades",
            DEV_PICO: "Lutron Keypads",
            DEV_GROUP: "Lutron Room Groups",
            DEV_TIMECLOCKEVENT: "Lutron Timeclock Events"
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
                    self.update_plugin_property(dev, PROP_ROOM, new_value=room)
                    return dev

        # Pick the folder for this device, create it if necessary

        if self.group_by == "Type":
            folderName = folderNameDict[devType]
            if folderName in indigo.devices.folders:
                theFolder = indigo.devices.folders[folderName].id
            else:
                self.logger.debug("Creating Device Folder: '{}'".format(folderName))
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

        self.logger.info("Creating {} device: '{}' ({}) in '{}'".format(devType, name, address, folderName))
        try:
            newDevice = indigo.device.create(indigo.kProtocol.Plugin, address=address, name=name, deviceTypeId=devType, props=props, folder=theFolder)
        except Exception as e:
            self.logger.error("Error calling indigo.device.create(): {}".format(e.message))
            newDevice = None

        return newDevice

    #################################
    #
    #  Future versions: implement additional thermostat actions, shades (define as dimmers for now)
