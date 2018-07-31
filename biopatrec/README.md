# BioPatRec Code for Quadcopter Control

## Setup
* Copy the `ArduPilot` folder into `BioPatRec/Control/`
* Perform Recording and Pattern Recognition with the following movements:
    * Open Hand
    * Close Hand
    * Flex Hand
    * Extend Hand
    * Pronation    (optional)
    * Supination   (optional)
    * Flex Elbow   (optional)
    * Extend Elbow (optional)
* Open `Real-time PatRec Mov2Mov`
* Under `Prosthetic device`, click `Select Folder`, and include this directory
* Select the `Wi-Fi` radio control, and input the following parameters:
    * `127.0.0.1` for the IP address
    * `8080` for the Port
* Click `Connect` and then `Test` to make sure you are connected
* Happy Flying!

## Testing Quadcopter Control
This codebase comes with a test GUI called `GUI_TestDroneKit.m` that allows
you to test the functions of the system without having myoelectric control
set up. It completely bypasses BioPatRec, and can be useful for first-time
set up and diagnostics.
