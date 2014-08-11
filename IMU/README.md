1.  Open the IMU.pde file using a program called "Processing", which can be downloaded here: https://processing.org/download/
    Ignore the data folder with the Universe file in it. That is used by the Processing sketch.
    
2.  Make sure cube satellite is plugged in using the arduino USB cable, and has the cubeSatIntegrated.ino sketch uploaded on the arduino.

3.  In the sktech under "SERIAL_PORT_NUM", there is a static integer that is the port number of the usb cable. The number of the usb can be found by opening up the arduino environment, selecting tools, hovering over Serial Port, and then count (starting from zero) from the top the Serial Ports until you reach the usb serial port. For example, if your list looks like the following: 

        dev/tty.Bluetooth-Incoming-Port     --> 0
        dev/cu.Bluetooth-Incoming-Port      --> 1
        dev/tty.Bluetooth-Modem             --> 2
        dev/cu.Bluetooth-Modem              --> 3
        dev/tty.usbserial-DA00STAW          --> 4
      X dev.tty.usbmodem1421                --> 5
        
    We would set the final static int SERIAL_PORT_NUMBER to the value of 5, because that is the port connected to the Arduino Mega on the CubeSat.
    
4.  Stablize the cubeSat on a flat surface and point the front of the cubeSat toward the computer screen.

5.  Click run a window with the message "Connecting to AGS" message should appear. If all goes well, in a few seconds a red box should appear on the screen. By turning the cubeSat, the animation on the window should move as well.

6.  If the sketch hangs the message "Wrong Port" appears in the console, make sure you have the sketch cubeSatIntegrated.ino on the arduino and the right port is selected in the SERIAL_PORT_NUM. Look at step 3 for more information. 