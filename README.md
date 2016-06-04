# Lutron RadioRA 2 / Caséta Plugin
Indigo plugin for Lutron RadioRa 2 and  Caséta

##Installation

1. Download the plugin zip file and double-click it to expand. Double-click the Lutron RadioRA 2 plugin to install it.

2. You need to connect your Mac to the main repeater’s serial port using a USB to serial adapter.  Although it costs more than most generic adaptors, I highly recommend the Keyspan adapter, which is solidly built and universally supported.  I also feel more comfortable hooking up a quality adapter to a repeater costing $400+!  You will also need a 9-pin serial cable (and possibly a serial to ethernet extender kit if your computer is more than 25 feet from the repeater).

Edit (3/2016):  Beginning with version 2.0.0, the plugin also supports Caseta Smartbridge PRO controllers.  You can connect your Indigo server to a RadioRA 2 repeater by either serial (still preferred) or IP Ethernet. Caseta Smart Bridge PRO only supports ethernet connection. The initial configuration dialog will prompt you to specify a serial port or IP connection. If you check the option for IP, you must enter a valid username and password. For Caseta systems, this is the telnet login, not the login you use for the Caseta app.

##Usage

1. Every RadioRA 2 device has a unique Integration ID, which you will need to identify it to Indigo.  You can get a text or XML file containing all the device IDs by running an integration report from the RadioRA 2 Essentials software or by downloading an XML file from the main repeater at this url: http://[Repeater IP Address]/DbXmlInfo.xml

2. Dimmers, Switches,  Fans and Thermostats can be added by clicking the New button on the Indigo Device panel and specifying the device’s integration ID.

3. A unique Indigo device type for shades is not defined in the initial plugin release, however shades can be assigned to dimmer devices.  100% represents fully open and 0% is fully closed.

4. In the current plugin implementation phantom buttons and phantom LEDs get assigned to the same Indigo device.  Button IDs are in the range of 1-100 and LED IDs are 101-200.  Each phantom button has a corresponding led.  For example, LED 101 is assigned to button 1.  To add a button/LED pair to Indigo, specify the integration ID of the LED.

5. Operating and monitoring status of the Lutron devices from Indigo should be self-explanatory.

##Limitations in plugin version 2.0.0

1. Main repeater phantom buttons can be set up in multiple ways but the plugin currently only supports On/Off commands.  So, for example, if a phantom button is defined as a toggle, clicking its On button in Indigo will turn the device on and clicking On again will turn it off.  Indigo’s Off button will have no effect on a phantom button that’s defined as a toggle.

1. The Lutron fan switches have four fan speeds (low, medium, medium-high, and high) but only three of these speeds are selectable from the Indigo GUI.
