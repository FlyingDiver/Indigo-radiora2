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
# 2.2.0 architectural update to normalize IP and Serial data flows.  Removed redundant code and execution paths.
# 2.2.1 debug statement fix
# 2.3.0 Fixed fan speeds.  Added button press events/triggers.
# 2.3.1 Fixed repository name
# 2.3.2 Fixed serial comms delay
# 2.3.3 Trigger processing changes
# 7.0.0 Indigo 7 logging and other API changes
# 7.0.1 Fixed trigger handling code
# 7.0.2 Added send raw command action

import serial
import socket
import telnetlib
import time
import select # was getting errors on the select.error exception in runConcurrentThread
import logging

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


    def startup(self):
        self.logger.info(u"Starting up Lutron")

        try:
            self.IP = self.pluginPrefs["IP"]
        except KeyError:
            self.logger.warning(u"Plugin not yet configured.\nPlease save the configuration then reload the plugin.\nThis should only happen the first time you run the plugin\nor if you delete the preferences file.")
            return

        self.updater = GitHubPluginUpdater(self)
        self.updater.checkForUpdate()
        self.updateFrequency = float(self.pluginPrefs.get('updateFrequency', 24)) * 60.0 * 60.0
        self.logger.debug(u"updateFrequency = " + str(self.updateFrequency))
        self.next_update_check = time.time()

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
        if dev.deviceTypeId == RA_PHANTOM_BUTTON:
            self.phantomButtons[dev.pluginProps[PROP_BUTTON]] = dev
            self.update_device_property(dev, "address", new_value = dev.pluginProps[PROP_BUTTON])
            self.logger.debug(u"Watching phantom button: " + dev.pluginProps[PROP_BUTTON])
        elif dev.deviceTypeId == RA_DIMMER:
            self.zones[dev.pluginProps[PROP_ZONE]] = dev
            self.update_device_property(dev, "address", new_value = dev.pluginProps[PROP_ZONE])
            self.logger.debug(u"Watching dimmer: " + dev.pluginProps[PROP_ZONE])
        elif dev.deviceTypeId == RA_SHADE:
            self.shades[dev.pluginProps[PROP_SHADE]] = dev
            self.update_device_property(dev, "address", new_value = dev.pluginProps[PROP_SHADE])
            dev.updateStateImageOnServer( indigo.kStateImageSel.None)
            self.logger.debug(u"Watching shade: " + dev.pluginProps[PROP_SHADE])
        elif dev.deviceTypeId == RA_SWITCH:
            self.switches[dev.pluginProps[PROP_SWITCH]] = dev
            self.update_device_property(dev, "address", new_value = dev.pluginProps[PROP_SWITCH])
            self.logger.debug(u"Watching switch: " + dev.pluginProps[PROP_SWITCH])
        elif dev.deviceTypeId == RA_FAN:
            self.fans[dev.pluginProps[PROP_FAN]] = dev
            self.update_device_property(dev, "address", new_value = dev.pluginProps[PROP_FAN])
            self.logger.debug(u"Watching fan: " + dev.pluginProps[PROP_FAN])
        elif dev.deviceTypeId == RA_THERMO:
            self.thermos[dev.pluginProps[PROP_THERMO]] = dev
            self.update_device_property(dev, "address", new_value = dev.pluginProps[PROP_THERMO])
            self.logger.debug(u"Watching thermostat: " + dev.pluginProps[PROP_THERMO])
        elif dev.deviceTypeId == RA_KEYPAD:
            self.keypads[dev.pluginProps[PROP_KEYPAD]+dev.pluginProps[PROP_KEYPADBUT]] = dev
            self.update_device_property(dev, "address", new_value = dev.pluginProps[PROP_KEYPAD] + "." + dev.pluginProps[PROP_KEYPADBUT])
            if int(dev.pluginProps[PROP_KEYPADBUT]) > 80:
                self.update_device_property(dev, "keypadButtonDisplayLEDState", new_value = dev.pluginProps[PROP_KEYPADBUT_DISPLAY_LED_STATE])
                self.logger.debug(u"Watching keypad: " + dev.pluginProps[PROP_KEYPAD] + " LED: " + dev.pluginProps[PROP_KEYPADBUT])
            else:
                self.update_device_property(dev, "keypadButtonDisplayLEDState", new_value = False)
                self.logger.debug(u"Watching keypad: " + dev.pluginProps[PROP_KEYPAD] + " button: " + dev.pluginProps[PROP_KEYPADBUT])
        elif dev.deviceTypeId == RA_SENSOR:
            self.sensors[dev.pluginProps[PROP_SENSOR]] = dev
            self.update_device_property(dev, "address", new_value = dev.pluginProps[PROP_SENSOR])
            self.logger.debug(u"Watching sensor: " + dev.pluginProps[PROP_SENSOR])
        elif dev.deviceTypeId == RA_CCI:
            self.ccis[dev.pluginProps[PROP_CCI_INTEGRATION_ID]+dev.pluginProps[PROP_COMPONENT]] = dev
            self.update_device_property(dev, "address", new_value = dev.pluginProps[PROP_CCI_INTEGRATION_ID] + "." + dev.pluginProps[PROP_COMPONENT])
            self.logger.debug(u"Watching CCI: " + dev.pluginProps[PROP_CCI_INTEGRATION_ID] + " input: " + dev.pluginProps[PROP_COMPONENT])
        elif dev.deviceTypeId == RA_CCO:
            self.ccos[dev.pluginProps[PROP_CCO_INTEGRATION_ID]] = dev
            self.update_device_property(dev, "address", new_value = dev.pluginProps[PROP_CCO_INTEGRATION_ID])
            ccoType = dev.pluginProps[PROP_CCO_TYPE]
            if ccoType == "momentary":
                dev.updateStateOnServer("onOffState", False)
            # To do - set SupportsStatusRequest to true if it is a sustained contact CCO
            #         haven't figured out a way to do that without hanging the UI when a new CCO is added
            self.logger.debug(u"Watching CCO: " + dev.pluginProps[PROP_CCO_INTEGRATION_ID])
        elif dev.deviceTypeId == RA_PICO:
            self.picos[dev.pluginProps[PROP_PICO_INTEGRATION_ID]+dev.pluginProps[PROP_PICOBUTTON]] = dev
            self.update_device_property(dev, "address", new_value = dev.pluginProps[PROP_PICO_INTEGRATION_ID] + "." + dev.pluginProps[PROP_PICOBUTTON])
            self.logger.debug(u"Watching Pico: " + dev.pluginProps[PROP_PICO_INTEGRATION_ID] + " button: " + dev.pluginProps[PROP_PICOBUTTON])

    def deviceStopComm(self, dev):
        if dev.deviceTypeId == RA_PHANTOM_BUTTON:
            del self.phantomButtons[dev.pluginProps[PROP_BUTTON]]
            self.logger.debug(u"Deleted phantom button: " + dev.pluginProps[PROP_BUTTON])
        elif dev.deviceTypeId == RA_DIMMER:
            del self.zones[dev.pluginProps[PROP_ZONE]]
            self.logger.debug(u"Deleted dimmer: " + dev.pluginProps[PROP_ZONE])
        elif dev.deviceTypeId == RA_SHADE:
            del self.shades[dev.pluginProps[PROP_SHADE]]
            self.logger.debug(u"Deleted shade: " + dev.pluginProps[PROP_SHADE])
        elif dev.deviceTypeId == RA_SWITCH:
            del self.switches[dev.pluginProps[PROP_SWITCH]]
            self.logger.debug(u"Deleted switch: " + dev.pluginProps[PROP_SWITCH])
        elif dev.deviceTypeId == RA_KEYPAD:
            del self.keypads[dev.pluginProps[PROP_KEYPAD]+dev.pluginProps[PROP_KEYPADBUT]]
            self.logger.debug(u"Deleted keypad: " + dev.pluginProps[PROP_KEYPAD]+dev.pluginProps[PROP_KEYPADBUT])
        elif dev.deviceTypeId == RA_FAN:
            del self.fans[dev.pluginProps[PROP_FAN]]
            self.logger.debug(u"Deleted fan: " + dev.pluginProps[PROP_FAN])
        elif dev.deviceTypeId == RA_THERMO:
            del self.thermos[dev.pluginProps[PROP_THERMO]]
            self.logger.debug(u"Deleted thermostat: " + dev.pluginProps[PROP_THERMO])
        elif dev.deviceTypeId == RA_SENSOR:
            del self.sensors[dev.pluginProps[PROP_SENSOR]]
            self.logger.debug(u"Deleted sensor: " + dev.pluginProps[PROP_SENSOR])
        elif dev.deviceTypeId == RA_CCI:
            del self.ccis[dev.pluginProps[PROP_CCI_INTEGRATION_ID]+dev.pluginProps[PROP_COMPONENT]]
            self.logger.debug(u"Deleted CCI: " + dev.pluginProps[PROP_CCI_INTEGRATION_ID]+dev.pluginProps[PROP_COMPONENT])
        elif dev.deviceTypeId == RA_CCO:
            del self.ccos[dev.pluginProps[PROP_CCO_INTEGRATION_ID]]
            self.logger.debug(u"Deleted CCO: " + dev.pluginProps[PROP_CCO_INTEGRATION_ID])
        elif dev.deviceTypeId == RA_PICO:
            del self.picos[dev.pluginProps[PROP_PICO_INTEGRATION_ID]+dev.pluginProps[PROP_PICOBUTTON]]
            self.logger.debug(u"Deleted Pico: " + dev.pluginProps[PROP_PICO_INTEGRATION_ID]+dev.pluginProps[PROP_PICOBUTTON])

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

                if (self.updateFrequency > 0.0) and (time.time() > self.next_update_check):
                    self.next_update_check = time.time() + self.updateFrequency
                    self.updater.checkForUpdate()

                if self.IP:
                    self.sleep(.1)
                    try:
                        if self.runstartup:
                            self.ipStartup()
                            self.runstartup = False

                        self._processCommand(self.connIP.read_until("\n", self.timeout))
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
        self.timeout = 1

        host = self.pluginPrefs["ip_address"]

        try:
            self.logger.info(u"Connecting via IP to %s" % host)
            self.connIP = telnetlib.Telnet(host, 23)

            a = self.connIP.read_until("\n", self.timeout)
            self.logger.debug(u"self.connIP.read: %s" % a)

            if 'login' in a:
                self.logger.debug(u"Sending username.")
                self.connIP.write(str(self.pluginPrefs["ip_username"]) + "\r\n")

                a = self.connIP.read_until("\n", self.timeout)
                self.logger.debug(u"self.connIP.read: %s" % a)
                if 'password' in a:
                    self.logger.debug(u"Sending password.")
                    self.connIP.write(str(self.pluginPrefs["ip_password"]) + "\r\n")
                else:
                    self.logger.debug(u"password failure.")
            else:
                self.logger.debug(u"username failure.")
            self.logger.debug(u"End of connection process.")

        except socket.error, e:
            self.logger.exception(u"Unable to connect to Lutron gateway. %s" % e.message)

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

            self.updateFrequency = float(self.pluginPrefs.get('updateFrequency', "24")) * 60.0 * 60.0
            self.logger.debug(u"updateFrequency = " + str(self.updateFrequency))
            self.next_update_check = time.time()

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
            self.connIP.write(str(cmd))
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
            level = cmdArray[3]

            if id in self.zones:
                zone = self.zones[id]
                if int(float(level)) == 0:
                    zone.updateStateOnServer("onOffState", False)
                else:
                    zone.updateStateOnServer("onOffState", True)
                    zone.updateStateOnServer("brightnessLevel", int(float(level)))
                self.logger.info(u"Received: Dimmer \"" + zone.name + "\" level set to " + str(level))
            elif id in self.shades:
                shade = self.shades[id]
                if int(float(level)) == 0:
                    shade.updateStateOnServer("onOffState", False)
                else:
                    shade.updateStateOnServer("onOffState", True)
                    shade.updateStateOnServer("brightnessLevel", int(float(level)))
                self.logger.info(u"Received: Shade \"" + shade.name + "\" opening set to " + str(level))
            elif id in self.switches:
                switch = self.switches[id]
                if int(float(level)) == 0:
                    switch.updateStateOnServer("onOffState", False)
                    self.logger.info(u"Received: Switch \"%s\" %s" % (switch.name, "turned Off"))
                else:
                    switch.updateStateOnServer("onOffState", True)
                    self.logger.info(u"Received: Switch \"%s\" %s" % (switch.name, "turned On"))
            elif id in self.ccos:
                cco = self.ccos[id]
                ccoType = cco.pluginProps[PROP_CCO_TYPE]
                if ccoType == "sustained":
                    if int(float(level)) == 0:
                     cco.updateStateOnServer("onOffState", False)
                    else:
                     cco.updateStateOnServer("onOffState", True)
                if level == '0.00':
                    self.logger.info(u"Received: CCO \"%s\" %s" % (cco.name, "Opened"))
                else:
                    self.logger.info(u"Received: CCO \"%s\" %s" % (cco.name, "Closed"))
            elif id in self.fans:
                fan = self.fans[id]
                if int(float(level)) == 0:
                    fan.updateStateOnServer("onOffState", False)
                else:
                    fan.updateStateOnServer("onOffState", True)
                    if float(level) < 26:
                        fan.updateStateOnServer("speedIndex", 1)
                    elif float(level) < 76:
                        fan.updateStateOnServer("speedIndex", 2)
                    else:
                        fan.updateStateOnServer("speedIndex", 3)
                self.logger.info(u"Received: Fan \"" + fan.name + "\" speed set to " + str(level))
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
            self.logger.debug(u"Received a phantom button status message: " + cmd)
            but = self.phantomButtons[id]
            if status == '0':
                but.updateStateOnServer("onOffState", False)
            elif status == '1':
                but.updateStateOnServer("onOffState", True)

        id = cmdArray[1]
        button = cmdArray[2]
        keypadid = id+button

        if keypadid in self.keypads:
            self.logger.debug(u"Received a keypad button/LED status message: " + cmd)
            dev = self.keypads[keypadid]
            keypad = self.keypads[keypadid]
            if status == '0':
                keypad.updateStateOnServer("onOffState", False)
            elif status == '1':
                keypad.updateStateOnServer("onOffState", True)

            if dev.pluginProps[PROP_KEYPADBUT_DISPLAY_LED_STATE]: # Also display this LED state on its corresponding button
                keypadid = keypadid[0:len(keypadid)-2] + keypadid[len(keypadid)-1] # Convert LED ID to button ID
                if keypadid in self.keypads:
                    keypad = self.keypads[keypadid]
                    self.logger.debug(u"Updating button status with state of LED for keypadID " + keypadid)
                    if int(status) == 0:
                        keypad.updateStateOnServer("onOffState", False)
                    elif int(status) == 1:
                        keypad.updateStateOnServer("onOffState", True)
                        self.logger.debug(u"Set status to True on Server.")
                else:
                    self.logger.error("WARNING: Invalid ID (%s) specified for LED.   Must be in range 81-87.  Please correct and reload the plugin." % keypadid)
                    self.logger.debug(keypadid)

            if action == '3': # Check for triggers
                self.debugLog(u"Received a Keypad Button press message, checking triggers: " + cmd)
                self.keypadTriggerCheck(id, button)

        if keypadid in self.picos:
            self.logger.debug(u"Received a pico button status message: " + cmd)
            but = self.picos[keypadid]
            if status == '0':
                but.updateStateOnServer("onOffState", False)
            elif status == '1':
                but.updateStateOnServer("onOffState", True)


        if keypadid in self.ccis:
            self.logger.debug(u"Received a CCI status message: " + cmd)
            cci = self.ccis[keypadid]
            if status == '0':
                cci.updateStateOnServer("onOffState", False)
                self.logger.info(u"Received: CCI %s %s" % (cci.name, "Opened"))
            elif status == '1':
                cci.updateStateOnServer("onOffState", True)
                self.logger.info(u"Received: CCI %s %s" % (cci.name, "Closed"))

        if id in self.sensors:
            self.logger.debug(u"Received a sensor status message: " + cmd)
            but = self.sensors[id]
            # self.logger.debug(u"Variable But: " + but)
            if status == '0':
                but.updateStateOnServer("onOffState", False)
                self.logger.info(u"Received: Motion Sensor %s %s" % (but.name, "vacancy detected"))
            elif status == '1':
                but.updateStateOnServer("onOffState", True)
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


    ##########################################

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


    #################################
    #
    #  Future versions: implement additional thermostat actions, shades (define as dimmers for now)

