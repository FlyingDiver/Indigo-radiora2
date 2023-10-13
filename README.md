# Lutron RadioRA 2 / Homeworks QS / Caséta Plugin
Indigo plugin for Lutron systems, including RadioRa 2 , RRa2 Select, Homeworks QS, and  Caséta

| Requirement            |                     |
|------------------------|---------------------|
| Minimum Indigo Version | 2022.1              |
| Python Library (API)   | Official            |
| Requires Local Network | Optional            |
| Requires Internet      | No                  |
| Hardware Interface     | Optional Serial     |

## Warning

This plugin is now Legacy status.  It will continue to be supported for Indigo changes, if needed, but no new features will be added.  The RRA2 and Homeworks devices supported by this plugin are no longer available from Lutron.  All other
devices are supported by the new Lutron Leap plugin.


## Requirements

1. For RadioRa2, a Main Repeater (RR-MAIN-REP-WH) with ethernet or serial connection to the Indigo server.

2. For RadioRa2 Select, a Main Repeater (RR-SEL-REP2) with ethernet.

3. For Homeworks QS, a HQP6-1 or HQP6-2 Processor.

3. For Caséta, a Wireless Smart Bridge Pro (L-BDGPRO2) with ethernet.


## Installation

1. Download the plugin zip file and double-click it to expand. Double-click the Lutron RadioRA 2 plugin to install it.

2. For RadioRa 2, you can connect to the main repeater either via Ethernet or a Serial using a USB to serial adapter.

3. For Homeworks QS, Caséta, or RadioRa2 Select, only Ethernet is supported.

4. The initial configuration dialog will prompt you to specify a serial port or IP connection. If you check the option for IP, you must enter a valid username and password. For Caseta systems, this is the telnet login, not the login you use for the Caseta app.  Foe Caseta, be sure to enable telnet support in the Advanced settings in the mobile app.

5. For Serial, select the connection type and device address in the Plugin configuration dialog.  Although it costs more than most generic adaptors, the Keyspan adapter, which is solidly built and universally supported, is highly recommended.



## Usage

1. Starting with version 7.2.0, this Plugin will automatically created Indigo devices to match your existing Lutron installation.  See the Wiki for more details.  This works for switches, dimmers, keypads, motion sensors, VCRX, and HVAC interfaces.

2. The Plugin will also create Indigo Devices to represent Room Groups and Timeclock Events.  You can also have it automatically create Triggers for these devices.

3. When creating triggers for different events, select the event or device from popup lists.  Knowing integration IDs is no longer required for trigger creation.

4. Creating devices manually does require the Integration ID (and Component ID). 

## Plugin Limitations

1. The Lutron fan switches have four fan speeds (low, medium, medium-high, and high) but only three of these speeds are selectable from the Indigo GUI.


### Indigo 7 Only


Version 7.0.0 and later of this plugin require Indigo 7 or greater.

**PluginID:**	com.jimandnoreen.indigoplugin.lutron-radiora2
