# Lutron RadioRA 2 / Caséta Plugin
Indigo plugin for Lutron RadioRa 2 and  Caséta

##Installation

1. Download the plugin zip file and double-click it to expand. Double-click the Lutron RadioRA 2 plugin to install it.

2. For RadioRa 2, you can connect to the main repeater either via Ethernet or a Serial using a USB to serial adapter.

3. For Caséta, only Ethernet is supported.

4. The initial configuration dialog will prompt you to specify a serial port or IP connection. If you check the option for IP, you must enter a valid username and password. For Caseta systems, this is the telnet login, not the login you use for the Caseta app.

5. For Serial, select the connection type and device address in the Plugin configuration dialog.  Although it costs more than most generic adaptors, the Keyspan adapter, which is solidly built and universally supported, is highly recommended.



##Usage

1. Starting with version 7.2.0, this Plugin will automatically created Indigo devices to match your existing RadioRa2 and Caséta installation.  See the Wiki for more details.  This works for switches, dimmers, keypads, motion sensors, VCRX, and HVAC interfaces.

2. The Plugin will also create Indigo Devices to represent Room Groups and Timeclock Events.  You can also have it automatically create Triggers for these devices.

3. When creating triggers for different events, select the event or device from popup lists.  Knowing integration IDs is no longer required for trigger creation.

4. Creating devices manually does require the Integration ID (and Component ID). 

##Plugin Limitations

1. The Lutron fan switches have four fan speeds (low, medium, medium-high, and high) but only three of these speeds are selectable from the Indigo GUI.


### Indigo 7 Only


Version 7.0.0 and later of this plugin require Indigo 7 or greater.

**PluginID:**	com.jimandnoreen.indigoplugin.lutron-radiora2
