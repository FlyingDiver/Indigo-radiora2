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
# This plugin is for Lutron RadioRA 2 and Caseta systems only and is NOT compatible with
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
# 2.0.0 added Caseta support by mathys and IP connectivity contributed by Sb08 and vic13.  Added menu option to query all devices
# 2.0.3 added Pico device type.  Changed CCI device type from relay to sensor.  Restrict query all devices to this plugin's devices.
# 2.1.0 added Group and TimeClock events.  Added BrightenBy and DimBy command support.  Added GitHubPluginUpdater support.
# 2.2.0 architectural update to normalize IP and Serial data flows.  Removed redundant code and execution paths.
# 2.2.1 debug statement fix
# 2.3.0 Fixed fan speeds.  Added button press events/triggers.
# 2.3.1 Fixed repository name
# 2.3.2 Fixed serial comms delay
# 2.3.3 Trigger processing changes
# 7.0.0 Indigo 7 logging and other API changes
# 7.0.1 Fixed trigger handling code
# 7.0.2 Added send raw command action
# 7.0.3 Added error trapping for corrupt ~OUTPUT strings
# 7.0.4 Plugin store release, no code changes
# 7.0.5 Error handling for socket connect failure, hide fade rate in dimmer config.  Merged Jim's 7.0.2.1 changes.

import serial
import socket
import telnetlib
import time
import select       # was getting errors on the select.error exception in runConcurrentThread
import logging

import requests
import xml.etree.ElementTree as ET
import threading

RA_MAIN_REPEATER = "ra2MainRepeater"
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
PROP_REPEATER = "repeater"
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

        pfmt = logging.Formatter('%(asctime)s.%(msecs)03d\t[%(levelname)8s] %(name)20s.%(funcName)-25s%(msg)s', datefmt='%Y-%m-%d %H:%M:%S')
        self.plugin_file_handler.setFormatter(pfmt)

        try:
            self.logLevel = int(self.pluginPrefs[u"logLevel"])
        except:
            self.logLevel = logging.WARNING
        self.indigo_log_handler.setLevel(self.logLevel)
        self.logger.debug(u"logLevel = " + str(self.logLevel))

        self.queryAtStartup = self.pluginPrefs.get(u"queryAtStartup", False)

        self.connSerial = {}
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
        self.IP = False     # Default to serial I/O, not IP -vic13
        self.portEnabled = False
        self.triggers = { }
        
        self.threadLock = threading.Lock()  # for background data fetch

    def startup(self):
        self.logger.info(u"Starting up Lutron")

        try:
            self.IP = self.pluginPrefs["IP"]
        except KeyError:
            self.logger.warning(u"Plugin not yet configured.\nPlease save the configuration then reload the plugin.\nThis should only happen the first time you run the plugin\nor if you delete the preferences file.")
            return

        if self.IP:
            self.ipStartup()
        else:
            self.serialStartup()
        self.runstartup = False

        if self.queryAtStartup:
            self.queryAllDevices()


    def shutdown(self):
        self.logger.info(u"Shutting down Lutron")
        if self.IP:
            self.connIP.close()
            
    ####################

    def triggerStartProcessing(self, trigger):
        self.logger.debug("Adding Trigger %s (%d)" % (trigger.name, trigger.id))
        assert trigger.id not in self.triggers
        self.triggers[trigger.id] = trigger

    def triggerStopProcessing(self, trigger):
        self.debugLog(u"Removing Trigger %s (%d)" % (trigger.name, trigger.id))
        assert trigger.id in self.triggers
        del self.triggers[trigger.id]

    def clockTriggerCheck(self, info):

        for triggerId, trigger in sorted(self.triggers.iteritems()):
            type = trigger.pluginTypeId
            try:
                eventNumber = trigger.pluginProps["eventNumber"]
            except KeyError:
                self.debugLog(u"Trigger %s (%s), Type: %s does not have eventNumber" % (trigger.name, trigger.id, type))

            if "timeClockEvent" != type:
                self.debugLog(u"\tSkipping Trigger %s (%s), wrong type: %s" % (trigger.name, trigger.id, type))
                continue

            if eventNumber != info:
                self.debugLog(u"\tSkipping Trigger %s (%s), wrong event: %s" % (trigger.name, trigger.id, info))
                continue

            self.debugLog(u"\tExecuting Trigger %s (%s), event: %s" % (trigger.name, trigger.id, info))
            indigo.trigger.execute(trigger)

    def groupTriggerCheck(self, info):

        for triggerId, trigger in sorted(self.triggers.iteritems()):
            type = trigger.pluginTypeId
            try:
                groupNumber = trigger.pluginProps["groupNumber"]
            except KeyError:
                self.debugLog(u"Trigger %s (%s), Type: %s does not have groupNumber" % (trigger.name, trigger.id, type))

            if "groupEvent" != type:
                self.debugLog(u"\tSkipping Trigger %s (%s), wrong type: %s" % (trigger.name, trigger.id, type))
                continue

            if groupNumber != info:
                self.debugLog(u"\tSkipping Trigger %s (%s), wrong group: %s" % (trigger.name, trigger.id, info))
                continue

            self.debugLog(u"\tExecuting Trigger %s (%s), group %s" % (trigger.name, trigger.id, groupNumber))
            indigo.trigger.execute(trigger)

    def keypadTriggerCheck(self, devID, compID):

        for triggerId, trigger in sorted(self.triggers.iteritems()):
            type = trigger.pluginTypeId
            try:
                deviceID = trigger.pluginProps["deviceID"]
            except KeyError:
                self.debugLog(u"Trigger %s (%s), Type: %s does not have deviceID" % (trigger.name, trigger.id, type))

            try:
                componentID = trigger.pluginProps["componentID"]
            except KeyError:
                self.debugLog(u"Trigger %s (%s), Type: %s does not have componentID" % (trigger.name, trigger.id, type))

            if "keypadButtonPress" != type:
                self.debugLog(u"\tSkipping Trigger %s (%s), wrong type: %s" % (trigger.name, trigger.id, type))
                continue

            if not (deviceID == devID and componentID == compID):
                self.debugLog(u"\tSkipping Trigger %s (%s), wrong keypad button: %s, %s" % (trigger.name, trigger.id, devID, compID))
                continue

            self.debugLog(u"\tExecuting Trigger %s (%s), keypad button: %s, %s" % (trigger.name, trigger.id, devID, compID))
            indigo.trigger.execute(trigger)

    ####################

    def update_device_property(self, dev, propertyname, new_value = ""):
        newProps = dev.pluginProps
        newProps.update ( {propertyname : new_value})
        dev.replacePluginPropsOnServer(newProps)
        return None

    ########################################

    def deviceStartComm(self, dev):
        if dev.deviceTypeId == RA_MAIN_REPEATER:
            pass
                        
        elif dev.deviceTypeId == RA_PHANTOM_BUTTON:
            if dev.pluginProps.get(PROP_REPEATER, None) == None:
                self.update_device_property(dev, PROP_REPEATER, new_value = "1")
                self.logger.info(u"%s: Added repeater property" % (dev.name))

            address = dev.pluginProps[PROP_REPEATER] + "." + dev.pluginProps[PROP_BUTTON]
            self.update_device_property(dev, "address", new_value = address)
            self.logger.debug(u"Watching phantom button: " + address)
            self.phantomButtons[address] = dev
            
        elif dev.deviceTypeId == RA_DIMMER:
            address = dev.pluginProps[PROP_ZONE]
            self.zones[address] = dev
            self.update_device_property(dev, "address", new_value = address)
            self.logger.debug(u"Watching dimmer: " + address)
            
        elif dev.deviceTypeId == RA_SHADE:
            address = dev.pluginProps[PROP_SHADE]
            self.shades[address] = dev
            self.update_device_property(dev, "address", new_value = address)
            dev.updateStateImageOnServer( indigo.kStateImageSel.None)
            self.logger.debug(u"Watching shade: " + address)
            
        elif dev.deviceTypeId == RA_SWITCH:
            address = dev.pluginProps[PROP_SWITCH]
            self.switches[address] = dev
            self.update_device_property(dev, "address", new_value = address)
            self.logger.debug(u"Watching switch: " + address)
            
        elif dev.deviceTypeId == RA_FAN:
            address = dev.pluginProps[PROP_FAN]
            self.fans[address] = dev
            self.update_device_property(dev, "address", new_value = address)
            self.logger.debug(u"Watching fan: " + address)

        elif dev.deviceTypeId == RA_THERMO:
            address = dev.pluginProps[PROP_THERMO]
            self.thermos[address] = dev
            self.update_device_property(dev, "address", new_value = address)
            self.logger.debug(u"Watching thermostat: " + address)

        elif dev.deviceTypeId == RA_KEYPAD:
            address = dev.pluginProps[PROP_KEYPAD] + "." + dev.pluginProps[PROP_KEYPADBUT]
            self.update_device_property(dev, "address", new_value = address)
            if int(dev.pluginProps[PROP_KEYPADBUT]) > 80:
                self.update_device_property(dev, "keypadButtonDisplayLEDState", new_value = dev.pluginProps[PROP_KEYPADBUT_DISPLAY_LED_STATE])
                self.logger.debug(u"Watching keypad: " + dev.pluginProps[PROP_KEYPAD] + " LED: " + dev.pluginProps[PROP_KEYPADBUT])
            else:
                self.update_device_property(dev, "keypadButtonDisplayLEDState", new_value = False)
                self.logger.debug(u"Watching keypad: " + dev.pluginProps[PROP_KEYPAD] + " button: " + dev.pluginProps[PROP_KEYPADBUT])
            self.keypads[address] = dev

        elif dev.deviceTypeId == RA_SENSOR:
            address = dev.pluginProps[PROP_SENSOR]
            self.sensors[address] = dev
            self.update_device_property(dev, "address", new_value = address)
            self.logger.debug(u"Watching sensor: " + address)

        elif dev.deviceTypeId == RA_CCI:
            address = dev.pluginProps[PROP_CCI_INTEGRATION_ID] + "." + dev.pluginProps[PROP_COMPONENT]
            self.ccis[address] = dev
            self.update_device_property(dev, "address", new_value = address)
            self.logger.debug(u"Watching CCI: " + address)

        elif dev.deviceTypeId == RA_CCO:
            address = dev.pluginProps[PROP_CCO_INTEGRATION_ID]
            self.ccos[address] = dev
            self.update_device_property(dev, "address", new_value = address)
            ccoType = dev.pluginProps[PROP_CCO_TYPE]
            if ccoType == "momentary":
                dev.updateStateOnServer("onOffState", False)
            # To do - set SupportsStatusRequest to true if it is a sustained contact CCO
            #         haven't figured out a way to do that without hanging the UI when a new CCO is added
            self.logger.debug(u"Watching CCO: " + address)

        elif dev.deviceTypeId == RA_PICO:
            address = dev.pluginProps[PROP_PICO_INTEGRATION_ID] + "." + dev.pluginProps[PROP_PICOBUTTON]
            self.picos[address] = dev
            self.update_device_property(dev, "address", new_value = address)
            self.logger.debug(u"Watching Pico: " + address)

        else:
            self.logger.error(u"deviceStartComm: Unknown device type:" + dev.deviceTypeId)
        
    def deviceStopComm(self, dev):
        if dev.deviceTypeId == RA_MAIN_REPEATER:
            pass
                        
        elif dev.deviceTypeId == RA_PHANTOM_BUTTON:
            address = dev.pluginProps[PROP_REPEATER] + "." + dev.pluginProps[PROP_BUTTON]
            del self.phantomButtons[address]
            self.logger.debug(u"Deleted phantom button: " + address)

        elif dev.deviceTypeId == RA_DIMMER:
            address = dev.pluginProps[PROP_ZONE]
            del self.zones[address]
            self.logger.debug(u"Deleted dimmer: " + address)

        elif dev.deviceTypeId == RA_SHADE:
            address = dev.pluginProps[PROP_SHADE]
            del self.shades[address]
            self.logger.debug(u"Deleted shade: " + address)

        elif dev.deviceTypeId == RA_SWITCH:
            address = dev.pluginProps[PROP_SWITCH]
            del self.switches[address]
            self.logger.debug(u"Deleted switch: " + address)

        elif dev.deviceTypeId == RA_KEYPAD:
            address = dev.pluginProps[PROP_KEYPAD] + "." + dev.pluginProps[PROP_KEYPADBUT]
            del self.keypads[address]
            self.logger.debug(u"Deleted keypad: " + address)

        elif dev.deviceTypeId == RA_FAN:
            address = dev.pluginProps[PROP_FAN]
            del self.fans[address]
            self.logger.debug(u"Deleted fan: " + address)

        elif dev.deviceTypeId == RA_THERMO:
            address = dev.pluginProps[PROP_THERMO]
            del self.thermos[address]
            self.logger.debug(u"Deleted thermostat: " + address)

        elif dev.deviceTypeId == RA_SENSOR:
            address = dev.pluginProps[PROP_SENSOR]
            del self.sensors[address]
            self.logger.debug(u"Deleted sensor: " + address)

        elif dev.deviceTypeId == RA_CCI:
            address = dev.pluginProps[PROP_CCI_INTEGRATION_ID] + "." + dev.pluginProps[PROP_COMPONENT]
            del self.ccis[address]
            self.logger.debug(u"Deleted CCI: " + address)

        elif dev.deviceTypeId == RA_CCO:
            address = dev.pluginProps[PROP_CCO_INTEGRATION_ID]
            del self.ccos[address]
            self.logger.debug(u"Deleted CCO: " + address)

        elif dev.deviceTypeId == RA_PICO:
            address = dev.pluginProps[PROP_PICO_INTEGRATION_ID] + "." + dev.pluginProps[PROP_PICOBUTTON]
            del self.picos[address]
            self.logger.debug(u"Deleted Pico: " + address)

        else:
            self.logger.error(u"deviceStopComm: Unknown device type:" + dev.deviceTypeId)

    def validateDeviceConfigUi(self, valuesDict, typeId, devId):

        errorsDict = indigo.Dict()

        if typeId == RA_KEYPAD and bool(valuesDict[PROP_KEYPADBUT_DISPLAY_LED_STATE]) and int(valuesDict[PROP_KEYPADBUT]) < 80:
            valuesDict[PROP_KEYPADBUT_DISPLAY_LED_STATE] = False
            self.logger.debug(u"validateDeviceConfigUi: forced PROP_KEYPADBUT_DISPLAY_LED_STATE to False for keypad # %s, button # %s" % (valuesDict[PROP_KEYPAD], valuesDict[PROP_KEYPADBUT]))

        if len(errorsDict) > 0:
            return (False, valuesDict, errorsDict)

        return (True, valuesDict)

    ########################################

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

        updateFrequency = int(valuesDict['updateFrequency'])
        if (updateFrequency < 0) or (updateFrequency > 24):
            errorDict['updateFrequency'] = u"Update frequency is invalid - enter a valid number (between 0 and 24)"
            self.logger.debug(u"updateFrequency out of range: " + valuesDict['updateFrequency'])


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
                self.logLevel = logging.WARNING
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
            elif 'GNET' in cmd:
                #command prompt is ready
                self.logger.debug(u"Command prompt received. Device is ready.")
            elif cmd != "!":
                self.logger.error(u"Unrecognized command: " + cmd)


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
        self.logger.debug(u"Received an Output message: " + cmd)
        cmdArray = cmd.split(',')
        id = cmdArray[1]
        action = cmdArray[2]
        
        if action == '1':  # set level
            try:
                level = float(cmdArray[3])
            except:
                self.logger.warning(u": Unable to parse level as float in _cmdOutputChange: " + str(cmdArray[3]))
                return
                
            if id in self.zones:
                zone = self.zones[id]
                if int(level) == 0:
                    zone.updateStateOnServer("onOffState", False)
                else:
                    zone.updateStateOnServer("onOffState", True)
                    zone.updateStateOnServer("brightnessLevel", int(level))
                self.logger.info(u"Received: Dimmer " + zone.name + " level set to " + str(level))
            elif id in self.shades:
                shade = self.shades[id]
                if int(level) == 0:
                    shade.updateStateOnServer("onOffState", False)
                else:
                    shade.updateStateOnServer("onOffState", True)
                    shade.updateStateOnServer("brightnessLevel", int(level))
                self.logger.info(u"Received: Shade " + shade.name + " opening set to " + str(level))
            elif id in self.switches:
                switch = self.switches[id]
                if int(level) == 0:
                    switch.updateStateOnServer("onOffState", False)
                    self.logger.info(u"Received: Switch %s %s" % (switch.name, "turned Off"))
                else:
                    switch.updateStateOnServer("onOffState", True)
                    self.logger.info(u"Received: Switch %s %s" % (switch.name, "turned On"))
            elif id in self.ccos:
                cco = self.ccos[id]
                ccoType = cco.pluginProps[PROP_CCO_TYPE]
                if ccoType == "sustained":
                    if int(level) == 0:
                     cco.updateStateOnServer("onOffState", False)
                    else:
                     cco.updateStateOnServer("onOffState", True)
                if level == 0.0:
                    self.logger.info(u"Received: CCO %s %s" % (cco.name, "Opened"))
                else:
                    self.logger.info(u"Received: CCO %s %s" % (cco.name, "Closed"))
            elif id in self.fans:
                fan = self.fans[id]
                if int(level) == 0:
                    fan.updateStateOnServer("onOffState", False)
                else:
                    fan.updateStateOnServer("onOffState", True)
                    if level < 26.0:
                        fan.updateStateOnServer("speedIndex", 1)
                    elif level < 76.0:
                        fan.updateStateOnServer("speedIndex", 2)
                    else:
                        fan.updateStateOnServer("speedIndex", 3)
                self.logger.info(u"Received: Fan " + fan.name + " speed set to " + str(level))
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
        else:
            self.logger.debug(u"Received Unknown Action Code:  %s" % action)
        return

    def _cmdDeviceChange(self,cmd):
        self.logger.debug(u"Received a Device message: " + cmd)

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

            # this code broken for new address format and for tabletop keypads (over 10 buttons)
            
            if dev.pluginProps[PROP_KEYPADBUT_DISPLAY_LED_STATE]: # Also display this LED state on its corresponding button
            
                keypadid = id + '.' + str(int(button) - 80)         # Convert LED ID to button ID
                if keypadid in self.keypads:
                    keypad = self.keypads[keypadid]
                    self.logger.debug(u"Updating button status with state of LED for keypadID " + keypadid)
                    if int(status) == 0:
                        keypad.updateStateOnServer("onOffState", False)
                    elif int(status) == 1:
                        keypad.updateStateOnServer("onOffState", True)
                        self.logger.debug(u"Set status to True on Server.")
                else:
                    self.logger.error("WARNING: Invalid ID (%s) specified for LED.   Must be ID of button + 80.  Please correct and reload the plugin." % keypadid)
                    self.logger.debug(keypadid)

            if action == '3': # Check for triggers
                self.debugLog(u"Received a Keypad Button press message, checking triggers: " + cmd)
                self.keypadTriggerCheck(id, button)

        if keypadid in self.picos:
            self.logger.debug(u"Received a pico button status message: " + cmd)
            dev = self.picos[keypadid]
            if status == '0':
                dev.updateStateOnServer("onOffState", False)
            elif status == '1':
                dev.updateStateOnServer("onOffState", True)


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
        self.clockTriggerCheck(event)

    def _cmdGroup(self,cmd):
        self.logger.debug(u"Received a Group message  " + cmd)
        cmdArray = cmd.split(',')
        id = cmdArray[1]
        action = cmdArray[2]
        status = cmdArray[3]
        self.clockTriggerCheck(status)


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
                phantom_button = dev.pluginProps[PROP_BUTTON]
                integration_id = dev.pluginProps[PROP_REPEATER]
                sendCmd = ("#DEVICE," + str(int(integration_id)) + ","+ str(int(phantom_button)-100) + ",3,") # Press button
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
                self.logger.debug(u"it is a cci")
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

            self.logger.info(u"sent \"%s\" %s" % (dev.name, "on"))

        ###### TURN OFF ######
        elif action.deviceAction == indigo.kDeviceAction.TurnOff:
            if dev.deviceTypeId == RA_PHANTOM_BUTTON:
                phantom_button = dev.pluginProps[PROP_BUTTON]
                integration_id = dev.pluginProps[PROP_REPEATER]
                sendCmd = ("#DEVICE," + str(int(integration_id)) + ","+ str(int(phantom_button)-100) + ",4,") # Release button
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
                self.logger.debug(u"it is a cci")
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
            self.logger.info(u"Sending: \"%s\" %s" % (dev.name, "off"))

        ###### TOGGLE ######
        elif action.deviceAction == indigo.kDeviceAction.Toggle:
            if dev.deviceTypeId == RA_PHANTOM_BUTTON:
                phantom_button = dev.pluginProps[PROP_BUTTON]
                integration_id = dev.pluginProps[PROP_REPEATER]
                sendCmd = ("#DEVICE," + str(int(integration_id)) + ","+ str(int(phantom_button)-100) + ",3,")
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
                self.logger.debug(u"it is a cci")
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
            self.logger.info(u"Sending: \"%s\" %s" % (dev.name, "toggle"))

        ###### SET BRIGHTNESS ######
        elif action.deviceAction == indigo.kDeviceAction.SetBrightness:
            if dev.deviceTypeId == RA_DIMMER:
                newBrightness = action.actionValue
                zone = dev.pluginProps[PROP_ZONE]
                sendCmd = ("#OUTPUT," + zone + ",1," + str(newBrightness))
                self.logger.info(u"Sending: \"%s\" %s to %d" % (dev.name, "set brightness", newBrightness))
            elif dev.deviceTypeId == RA_SHADE:
                newBrightness = action.actionValue
                shade = dev.pluginProps[PROP_SHADE]
                sendCmd = ("#OUTPUT," + shade + ",1," + str(newBrightness))
                self.logger.info(u"Sending: \"%s\" %s to %d" % (dev.name, "set shade open to", newBrightness))

        ###### BRIGHTEN BY ######
        elif action.deviceAction == indigo.kDimmerRelayAction.BrightenBy:
            newBrightness = dev.brightness + action.actionValue
            if newBrightness > 100:
                newBrightness = 100

            if dev.deviceTypeId == RA_DIMMER:
                zone = dev.pluginProps[PROP_ZONE]
                sendCmd = ("#OUTPUT," + zone + ",1," + str(newBrightness))
                self.logger.info(u"Sending: \"%s\" %s to %d" % (dev.name, "set brightness", newBrightness))
            elif dev.deviceTypeId == RA_SHADE:
                shade = dev.pluginProps[PROP_SHADE]
                sendCmd = ("#OUTPUT," + shade + ",1," + str(newBrightness))
                self.logger.info(u"Sending: \"%s\" %s to %d" % (dev.name, "set shade open to", newBrightness))

        ###### DIM BY ######
        elif action.deviceAction == indigo.kDimmerRelayAction.DimBy:
            newBrightness = dev.brightness - action.actionValue
            if newBrightness < 0:
                newBrightness = 0

            if dev.deviceTypeId == RA_DIMMER:
                zone = dev.pluginProps[PROP_ZONE]
                sendCmd = ("#OUTPUT," + zone + ",1," + str(newBrightness))
                self.logger.info(u"Sending: \"%s\" %s to %d" % (dev.name, "set brightness", newBrightness))
            elif dev.deviceTypeId == RA_SHADE:
                shade = dev.pluginProps[PROP_SHADE]
                sendCmd = ("#OUTPUT," + shade + ",1," + str(newBrightness))
                self.logger.info(u"Sending: \"%s\" %s to %d" % (dev.name, "set shade open to", newBrightness))

        ###### STATUS REQUEST ######
        elif action.deviceAction == indigo.kDeviceAction.RequestStatus:
            if dev.deviceTypeId == RA_PHANTOM_BUTTON:
                phantom_button = dev.pluginProps[PROP_BUTTON]
                integration_id = dev.pluginProps[PROP_REPEATER]
                sendCmd = ("?DEVICE," + str(int(integration_id)) + ","+ str(int(phantom_button)) + ",9,")
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
                self.logger.info(u"This device does not respond to Status Requests")
            elif dev.deviceTypeId == RA_CCO:
                cco = dev.pluginProps[PROP_CCO_INTEGRATION_ID]
                ccoType = dev.pluginProps[PROP_CCO_TYPE]
                if ccoType == "momentary":
                    self.logger.info(u"Momentary CCOs do not respond to Status Requests")
                else:
                    sendCmd = ("?OUTPUT," + cco + ",1,")


        self._sendCommand(sendCmd)
        self.logger.debug(u"Sent: \"%s\" %s %s" % (dev.name, dev.onState, sendCmd))

    ######################
    # Sensor Action callback
    ######################
    def actionControlSensor(self, action, dev):
        self.logger.info(u"This device does not respond to Status Requests")

    ######################
    # Fan Action callback
    ######################
    def actionControlSpeedControl(self, action, dev):

        ###### SET SPEED ######
        if action.speedControlAction == indigo.kSpeedControlAction.SetSpeedIndex:
            if dev.deviceTypeId == RA_FAN:
                newSpeed = action.actionValue
                fan = dev.pluginProps[PROP_FAN]
                self.logger.info(u"Sending: \"%s\" %s to %d" % (dev.name, "set fan speed", newSpeed))
                if newSpeed == 0:
                    self._sendCommand("#OUTPUT," + fan + ",1,0")
                elif newSpeed == 1:
                    self._sendCommand("#OUTPUT," + fan + ",1,25")
                elif newSpeed == 2:
                    self._sendCommand("#OUTPUT," + fan + ",1,75")
                else:
                    self._sendCommand("#OUTPUT," + fan + ",1,100")

        ###### STATUS REQUEST ######
        elif action.speedControlAction == indigo.kSpeedControlAction.RequestStatus:
                self.logger.info(u"Sending: \"%s\" %s" % (dev.name, "status request"))
                integration_id = dev.pluginProps[PROP_FAN]
                self._sendCommand("?OUTPUT," + integration_id + ",1,")

        ###### CYCLE SPEED ######
        # Future enhancement
        #elif action.speedControlAction == indigo.kSpeedControlAction.cycleSpeedControlState:

        ###### TOGGLE ######
        # Future enhancement
        #elif action.speedControlAction == indigo.kSpeedControlAction.toggle:
        #self.logger.info(u"sent \"%s\" %s" % (dev.name, "cycle speed"))

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
                self.logger.info(u"sent \"%s\" %s" % (dev.name, "set hvac mode to Off"))
            elif mode == indigo.kHvacMode.Heat:
                self._sendCommand("#HVAC," + integration_id + ",3,2")
                self.logger.info(u"sent \"%s\" %s" % (dev.name, "set hvac mode to Heat"))
            elif mode == indigo.kHvacMode.Cool:
                self._sendCommand("#HVAC," + integration_id + ",3,3")
                self.logger.info(u"sent \"%s\" %s" % (dev.name, "set hvac mode to Cool"))
            elif mode == indigo.kHvacMode.HeatCool:
                self._sendCommand("#HVAC," + integration_id + ",3,4")
                self.logger.info(u"sent \"%s\" %s" % (dev.name, "set hvac mode to Auto"))

        ###### SET FAN MODE ######
        elif action.thermostatAction == indigo.kThermostatAction.SetFanMode:
            mode = action.actionMode
            if mode == indigo.kFanMode.Auto:
                self.logger.info(u"Sending: \"%s\" %s" % (dev.name, "set fan mode to Auto"))
                self._sendCommand("#HVAC," + integration_id + ",4,1")
            elif mode == indigo.kFanMode.AlwaysOn:
                self.logger.info(u"Sending: \"%s\" %s" % (dev.name, "set fan mode to Always On"))
                self._sendCommand("#HVAC," + integration_id + ",4,2")

        ###### STATUS REQUEST ######
        elif action.thermostatAction == indigo.kThermostatAction.RequestStatusAll:
            self.logger.debug(u"Sending: \"%s\" %s" % (dev.name, "status request"))
            self._sendCommand("?HVAC," + integration_id + ",1,") # get temperature
            self._sendCommand("?HVAC," + integration_id + ",2,") # get heat and cool setpoints
            self._sendCommand("?HVAC," + integration_id + ",3,") # get operating mode
            self._sendCommand("?HVAC," + integration_id + ",4,") # get fan mode


    ########################################
    # Plugin Actions object callbacks (pluginAction is an Indigo plugin action instance)

    def fadeDimmer(self, pluginAction, dimmerDevice):

        brightness =  indigo.activePlugin.substitute(pluginAction.props["brightness"])
        fadeTime =  indigo.activePlugin.substitute(pluginAction.props["fadeTime"])
        zone = dimmerDevice.address

        sendCmd = ("#OUTPUT," + zone + ",1," + str(brightness) + "," + str(fadeTime))
        self.logger.info(u"Sending: \"%s\" set brightness to %s with fade %s" % (dimmerDevice.name, brightness, fadeTime))
        self._sendCommand(sendCmd)

    def sendRawCommand(self, pluginAction, dimmerDevice):

        sendCmd =  indigo.activePlugin.substitute(pluginAction.props["commandString"])

        self.logger.info(u"Sending Raw Command: \"%s\"" % sendCmd)
        self._sendCommand(sendCmd)

    def sendRawCommandMenu(self, valuesDict, typeId):

        sendCmd =  indigo.activePlugin.substitute(valuesDict["commandString"])

        self.logger.info(u"Sending Raw Command (Menu): \"%s\"" % sendCmd)
        self._sendCommand(sendCmd)

        return True


    def createAllDevicesMenu(self, valuesDict, typeId):

        if not self.IP:
            self.logger.warning(u"Unable to create devices, no IP connection to repeater.")
            return False
            
        deviceThread = threading.Thread(target = self.createAllDevices, args = (valuesDict, ))
        deviceThread.start()    
                
        return True        

    def createAllDevices(self, valuesDict):
        
        if not self.threadLock.acquire(False):
            self.logger.warning(u"Unable to create devices, process already running.")
            return

        self.group_by = valuesDict["group_by"]
        self.simulated = bool(valuesDict["simulated"])
        self.create_unused = bool(valuesDict["create_unused"])
        self.use_local = bool(valuesDict["use_local"])

        self.logger.info(u"Creating Devices from repeater at %s, Grouping = %s, Simulated = %s, Create Unprogrammed = %s" % (self.pluginPrefs["ip_address"], self.group_by, self.simulated, self.create_unused))

        if self.use_local:
            tree = ET.parse('/Users/jkeenan/Projects/Indigo PlugIns/Lutron/DbXmlInfo.xml')
            root = tree.getroot()
        else:

            login = 'http://' + self.pluginPrefs["ip_address"] + '/login?login=lutron&password=lutron'
            fetch = 'http://' + self.pluginPrefs["ip_address"] + '/DbXmlInfo.xml'

            self.logger.info(u"Creating Devices - starting data fetch...")
        
            s = requests.Session()
            r = s.get(login)
            r = s.get(fetch)
            root = ET.fromstring(r.text)        

        self.logger.info(u"Creating Devices fetch completed, parsing data...")
        
        for room in root.findall('Areas/Area/Areas/Area'):
            self.logger.debug("Room: %s (%s)" % (room.attrib['Name'], room.attrib['IntegrationID']))
        
            for device in room.findall('DeviceGroups/Device'):
                self.logger.debug("\tDevice: %s (%s,%s)" % (device.attrib['Name'], device.attrib['IntegrationID'], device.attrib['DeviceType']))
                if device.attrib['DeviceType'] == "MAIN_REPEATER":
                    for component in device.findall('Components/Component'):
                        self.logger.debug("\t\tComponent: %s (%s)" % (component.attrib['ComponentNumber'], component.attrib['ComponentType']))
                        if component.attrib['ComponentType'] == "BUTTON":
                            assignments = len(component.findall('Button/Actions/Action/Presets/Preset/PresetAssignments/PresetAssignment'))                            
                            name = "Repeater " + device.attrib['IntegrationID'] + " - Button " + component.attrib['ComponentNumber']
                            button = str(int(component.attrib['ComponentNumber']) + 100)
                            address = device.attrib['IntegrationID'] + "." + button
                            if not self.create_unused and assignments == 0:
                                self.logger.info("Skipping button creation, %d PresetAssignments: '%s' (%s)" % (assignments, name, address))
                                continue
                            props={ 'repeater': device.attrib['IntegrationID'], 'button': button, 'notes': ""}
                            self.createLutronDevice(RA_PHANTOM_BUTTON, name, address, props, room.attrib['Name'])

                        elif component.attrib['ComponentType'] == "LED":
                            pass

                        else:
                            self.logger.error("Unexpected Component Type: %s (%s)" % (component.attrib['Name'], component.attrib['ComponentType']))

                else:
                    self.logger.error("Unexpected Device Type: %s (%s)" % (device.attrib['Name'], device.attrib['DeviceType']))


            for output in room.findall('Outputs/Output'):
                self.logger.debug("\tOutput: %s (%s) %s" % (output.attrib['Name'], output.attrib['IntegrationID'], output.attrib['OutputType']))

                if output.attrib['OutputType'] == "INC" or output.attrib['OutputType'] == "MLV" or output.attrib['OutputType'] == "AUTO_DETECT":
                    name = room.attrib['Name'] + " - " + output.attrib['Name']
                    props={ 'zone': output.attrib['IntegrationID'], 'fadeTime': "1", 'notes': ""}
                    self.createLutronDevice(RA_DIMMER, name, output.attrib['IntegrationID'], props, room.attrib['Name'])
                    
                elif output.attrib['OutputType'] == "NON_DIM":
                    name = room.attrib['Name'] + " - " + output.attrib['Name']
                    props={ 'switch': output.attrib['IntegrationID'], 'notes': ""}
                    self.createLutronDevice(RA_SWITCH, name, output.attrib['IntegrationID'], props, room.attrib['Name'])
                        
                elif output.attrib['OutputType'] == "SYSTEM_SHADE":
                    name = room.attrib['Name'] + " - " + output.attrib['Name']
                    props={ 'shade': output.attrib['IntegrationID'], 'notes': ""}
                    self.createLutronDevice(RA_SHADE, name, output.attrib['IntegrationID'], props, room.attrib['Name'])

                elif output.attrib['OutputType'] == "CEILING_FAN_TYPE":
                    name = room.attrib['Name'] + " - " + output.attrib['Name']
                    props={ 'fan': output.attrib['IntegrationID'], 'notes': ""}
                    self.createLutronDevice(RA_FAN, name, output.attrib['IntegrationID'], props, room.attrib['Name'])

                elif output.attrib['OutputType'] == "CCO_PULSED":
                    name = room.attrib['Name'] + " - " + output.attrib['Name']
                    props={ 'ccoIntegrationID': output.attrib['IntegrationID'], 'ccoType': "momentary", 'notes': ""}
                    self.createLutronDevice(RA_CCO, name, output.attrib['IntegrationID'], props, room.attrib['Name'])

                elif output.attrib['OutputType'] == "CCO_MAINTAINED":
                    name = room.attrib['Name'] + " - " + output.attrib['Name']
                    props={ 'ccoIntegrationID': output.attrib['IntegrationID'], 'ccoType': "sustained", 'notes': "", 'SupportsStatusRequest': "False"}
                    self.createLutronDevice(RA_CCO, name, output.attrib['IntegrationID'], props, room.attrib['Name'])

                else:
                    self.logger.error("Unexpected Output Type: %s (%s)" % (output.attrib['Name'], output.attrib['OutputType']))


            for device in room.findall('DeviceGroups/DeviceGroup/Devices/Device'):
                self.logger.debug("\tDevice: %s (%s,%s)" % (device.attrib['Name'], device.attrib['IntegrationID'], device.attrib['DeviceType']))
    
                if device.attrib['DeviceType'] == "SEETOUCH_KEYPAD" or device.attrib['DeviceType'] == "HYBRID_SEETOUCH_KEYPAD" or device.attrib['DeviceType'] == "SEETOUCH_TABLETOP_KEYPAD":  
                    for component in device.findall('Components/Component'):
                        self.logger.debug("\t\tComponent: %s (%s)" % (component.attrib['ComponentNumber'], component.attrib['ComponentType']))
                        if component.attrib['ComponentType'] == "BUTTON":
                            assignments = len(component.findall('Button/Actions/Action/Presets/Preset/PresetAssignments/PresetAssignment'))  
                            try:                          
                                engraving = component.find("Button").attrib['Engraving']
                            except:
                                name = room.attrib['Name'] + " - " + device.attrib['Name'] + " - " + component.find('Button').get('Name')
                            else:
                                name = room.attrib['Name'] + " - " + device.attrib['Name'] + " - " + engraving
                            address = device.attrib['IntegrationID'] + "." + component.attrib['ComponentNumber']
                            if not self.create_unused and assignments == 0:
                                self.logger.info("Skipping button creation, %d PresetAssignments: '%s' (%s)" % (assignments, name, address))
                                continue
                            props = { 'listType': "button", 'keypad': device.attrib['IntegrationID'], 'keypadButton': component.attrib['ComponentNumber'], "keypadButtonDisplayLEDState": "false" }
                            self.createLutronDevice(RA_KEYPAD, name, address, props, room.attrib['Name'])
                            
                            # always create button LED
                            
                            name = name + " LED"  
                            keypadLED = str(int(component.attrib['ComponentNumber']) + 80)
                            address = device.attrib['IntegrationID'] + "." + keypadLED
                            props = { 'listType': "LED", 'keypad': device.attrib['IntegrationID'], 'keypadButton': keypadLED, "keypadButtonDisplayLEDState": "false" }
                            self.createLutronDevice(RA_KEYPAD, name, address, props, room.attrib['Name'])

                        elif component.attrib['ComponentType'] == "LED":
                            pass    # LED device created same time as button
                            
                        else:
                            self.logger.error("Unexpected Component Type: %s (%s)" % (component.attrib['Name'], component.attrib['ComponentType']))
                                                 
                elif device.attrib['DeviceType'] == "VISOR_CONTROL_RECEIVER":
                    for component in device.findall('Components/Component'):
                        self.logger.debug("\t\tComponent: %s (%s)" % (component.attrib['ComponentNumber'], component.attrib['ComponentType']))
                        if component.attrib['ComponentType'] == "BUTTON":
                            assignments = len(component.findall('Button/Actions/Action/Presets/Preset/PresetAssignments/PresetAssignment'))                            
                            name = room.attrib['Name'] + " - " + device.attrib['Name'] + " - Button " + component.attrib['ComponentNumber']
                            address = device.attrib['IntegrationID'] + "." + component.attrib['ComponentNumber']
                            if not self.create_unused and assignments == 0:
                                self.logger.info("Skipping button creation, %d PresetAssignments: '%s' (%s)" % (assignments, name, address))
                                continue
                            props = { 'listType': "button", 'keypad': device.attrib['IntegrationID'], 'keypadButton': component.attrib['ComponentNumber'], "keypadButtonDisplayLEDState": "false" }
                            self.createLutronDevice(RA_KEYPAD, name, address, props, room.attrib['Name'])

                        elif component.attrib['ComponentType'] == "LED":
                            button = str(int(component.attrib['ComponentNumber']) - 80)
                            name = room.attrib['Name'] + " - " + device.attrib['Name'] + " - Button " + button + " LED"
                            address = device.attrib['IntegrationID'] + "." + component.attrib['ComponentNumber']
                            props = { 'listType': "LED", 'keypad': device.attrib['IntegrationID'], 'keypadButton': component.attrib['ComponentNumber'], "keypadButtonDisplayLEDState": "false" }
                            self.createLutronDevice(RA_KEYPAD, name, address, props, room.attrib['Name'])

                        elif component.attrib['ComponentType'] == "CCI":
                            name = room.attrib['Name'] + " - CCI " + device.attrib['Name'] + " - Input " + component.attrib['ComponentNumber']
                            address = device.attrib['IntegrationID'] + "." + component.attrib['ComponentNumber']
                            props = { 'cciIntegrationID': device.attrib['IntegrationID'], 'component': component.attrib['ComponentNumber'], 'notes': "", "SupportsStatusRequest": "False" }
                            self.createLutronDevice(RA_CCI, name, address, props, room.attrib['Name'])

                        else:
                            self.logger.error("Unexpected Component Type: %s (%s)" % (component.attrib['Name'], component.attrib['ComponentType']))
                                                 
                elif device.attrib['DeviceType'] == "PICO_KEYPAD":
                    for component in device.findall('Components/Component'):
                        self.logger.debug("\t\tComponent: %s (%s)" % (component.attrib['ComponentNumber'], component.attrib['ComponentType']))
                        if component.attrib['ComponentType'] == "BUTTON":
                            assignments = len(component.findall('Button/Actions/Action/Presets/Preset/PresetAssignments/PresetAssignment'))                            
                            try:                          
                                engraving = component.find("Button").attrib['Engraving']
                            except:
                                name = room.attrib['Name'] + " - " + device.attrib['Name'] + " - " + component.find('Button').get('Name')
                            else:
                                name = room.attrib['Name'] + " - " + device.attrib['Name'] + " - " + engraving
                            address = device.attrib['IntegrationID'] + "." + component.attrib['ComponentNumber']
                            if not self.create_unused and assignments == 0:
                                self.logger.info("Skipping button creation, %d PresetAssignments: '%s' (%s)" % (assignments, name, address))
                                continue
                            props={ 'picoIntegrationID': device.attrib['IntegrationID'], 'picoButton': component.attrib['ComponentNumber'], 'notes': ""}
                            self.createLutronDevice(RA_PICO, name, address, props, room.attrib['Name'])

                        else:
                            self.logger.error("Unexpected Component Type: %s (%s)" % (component.attrib['Name'], component.attrib['ComponentType']))
                                                 
                elif device.attrib['DeviceType'] == "MOTION_SENSOR" or device.attrib['DeviceType'] == "CCI":
                    for component in device.findall('Components/Component'):
                        self.logger.debug("\t\tComponent: %s (%s)" % (component.attrib['ComponentNumber'], component.attrib['ComponentType']))
                        if component.attrib['ComponentType'] == "CCI":
                            name = room.attrib['Name'] + " - Motion Sensor " + device.attrib['Name']
                            address = device.attrib['IntegrationID']
                            props = { 'sensor': device.attrib['IntegrationID'], 'notes': "", "SupportsStatusRequest": "False" }
                            self.createLutronDevice(RA_SENSOR, name, address, props, room.attrib['Name'])

                        else:
                            self.logger.error("Unexpected Component Type: %s (%s)" % (component.attrib['Name'], component.attrib['ComponentType']))
                                                 
                else:
                    self.logger.error("Unexpected Device Type: %s (%s)" % (device.attrib['Name'], device.attrib['DeviceType']))
                               
                        
        self.logger.info(u"Creating Devices done. Doing query all.")
        self.queryAllDevices()
        self.threadLock.release()

        return


    def createLutronDevice(self, devType, name, address, props, room, notes = None):

        folderNameDict = {
            RA_MAIN_REPEATER    : "Lutron Repeaters",
            RA_PHANTOM_BUTTON   : "Lutron Phantom Buttons",
            RA_DIMMER           : "Lutron Dimmers",
            RA_SWITCH           : "Lutron Switches",
            RA_KEYPAD           : "Lutron Keypad Buttons",
            RA_FAN              : "Lutron Fans",
            RA_THERMO           : "Lutron Thermostats",
            RA_SENSOR           : "Lutron Sensors",
            RA_CCO              : "Lutron Switches",
            RA_CCI              : "Lutron Sensors",
            RA_SHADE            : "Lutron Shades",
            RA_PICO             : "Lutron Keypad Buttons"
        }

        # first, make sure this device doesn't exist.  Unless I screwed up, the addresses should be unique
        # it would be more efficient to search through the internal device lists, but a pain to code.
        
        for dev in indigo.devices.iter("self"):
            if dev.address == address:
                self.logger.warning("Not creating duplicate device: '%s' (%s)" % (name, address))
                return
                
            
        if self.group_by == "Type":
            folderName = folderNameDict[devType]
            if folderName in indigo.devices.folders:
                theFolder = indigo.devices.folders[folderName].id
            else:
                theFolder = indigo.devices.folder.create(folderName).id
        elif self.group_by == "Room":
            folderName = "Lutron " + room
            if folderName in indigo.devices.folders:
                theFolder = indigo.devices.folders[folderName].id
            else:
                theFolder = indigo.devices.folder.create(folderName).id
        else:
            folderName = "DEVICES"
            theFolder = 0
                    
        if self.simulated:
            self.logger.info("Not creating %s device: '%s' (%s) in '%s'" % (devType, name, address, folderName))
        else:
            self.logger.info("Creating %s device: '%s' (%s) in '%s'" % (devType, name, address, folderName))
            try:
                newdev = indigo.device.create(indigo.kProtocol.Plugin, address=address, name=name, deviceTypeId=devType, props=props, folder=theFolder)
            except Exception, e:
                self.logger.error("Error calling indigo.device.create(): %s" % (e.message))
                                                    
        return
        

    #################################
    #
    #  Future versions: implement additional thermostat actions, shades (define as dimmers for now)

