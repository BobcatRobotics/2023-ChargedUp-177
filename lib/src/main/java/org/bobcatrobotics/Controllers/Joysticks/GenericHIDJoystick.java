package org.bobcatrobotics.Controllers.Joysticks;

import java.util.HashMap;
import java.util.Map;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class GenericHIDJoystick implements JoystickBase{
    private final GenericHID controller;
    private final Alert joystickUnpluggedAlert;
    private Map<String , Integer> buttonMap = new HashMap<>();

    public GenericHIDJoystick(int port, String name){
        joystickUnpluggedAlert =
      new Alert(name + " Joystick unplugged!", AlertType.kWarning);
        controller = new GenericHID(port);
                buttonMap = Map.ofEntries(
            Map.entry("A",1)
        );
    }
    public Trigger getRawButton( String buttonName ){
        return new Trigger(()->controller.getRawButton(buttonMap.get(buttonName).intValue()));
    }
    public double getStickX(){
        return controller.getRawAxis(1);
    }
    public double getStickY(){
        return controller.getRawAxis(2);
    }
    public double getStickZ(){
        return controller.getRawAxis(3);
    }
    public double getFineStickX(){
        return controller.getRawAxis(1);
    }
    public double getFineStickY(){
        return controller.getRawAxis(2);
    }
    public double getFineStickZ(){
        return controller.getRawAxis(2);
    }
    public void updateControllerAlerts() {
        joystickUnpluggedAlert.set(!controller.isConnected());
    }
}

