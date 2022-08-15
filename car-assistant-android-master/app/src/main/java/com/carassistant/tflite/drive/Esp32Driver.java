package com.carassistant.tflite.drive;

import com.carassistant.managers.BTEsp32;

public class Esp32Driver {
    // variables to include : 1. BTEp32 to send commands over bluetooth to ESP32
    // 2. BTEsp32 send data over bluetooth to ESP32
    // connect the ESP32 to the car and start the car
    // send commands to the ESP32 to start the car
    public static BTEsp32 btEsp32;
    // default constructor
    public Esp32Driver() {
        this.btEsp32 = new BTEsp32();
    }
    // constructor with parameters
    public Esp32Driver(BTEsp32 btEsp32) {
        this.btEsp32 = btEsp32;
    }
    // function to send command to ESP32
    public static boolean sendCommand(String command) {
        return btEsp32.writeBt(command.getBytes());
    }
}
