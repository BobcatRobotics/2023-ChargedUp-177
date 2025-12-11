package org.bobcatrobotics.Controllers.Gamepads;

import edu.wpi.first.wpilibj2.command.button.Trigger;

import java.util.HashMap;
import java.util.Map;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;

public class EightBitDoController  implements ControllerBase{
    CommandJoystick controller;
    private final Alert controllerUnpluggedAlert;
    private Map<String , Integer> buttonMap = new HashMap<>();

    public EightBitDoController(int port, String name){
        controllerUnpluggedAlert =
        new Alert(name + " Joystick unplugged!", AlertType.kWarning);
        controller = new CommandJoystick(port);
        buttonMap = Map.ofEntries(
            Map.entry("A",2),
            Map.entry("B",1),
            Map.entry("X",4),
            Map.entry("Y",3),
            Map.entry("Select",7),
            Map.entry("Start",8)
        );
    }
    public Trigger getButton(String buttonName){
        return controller.button(buttonMap.get(buttonName).intValue());
    }
    public Trigger getLeftTrigger(){
        return controller.button(9);
    }
    public Trigger getRightTrigger(){
        return controller.button(10);
    }
    public Trigger getLeftBumper(){
        return controller.button(5);
    }
    public Trigger getRightBumper(){
        return controller.button(6);
    }
    public double getLeftX(){
        return controller.getRawAxis(0);
    }
    public double getRightX(){
        return controller.getRawAxis(4);
    }
    public double getLeftY(){
        return -controller.getRawAxis(1);
    }
    public double getRightY(){
        return -controller.getRawAxis(3);
    }
    public void updateControllerAlerts() {
        controllerUnpluggedAlert.set(!controller.isConnected());
    }
    public Trigger getPovUp(){
        return controller.povUp();
    }
    public Trigger getPovDown(){
        return controller.povDown();
    }
    public Trigger getPovLeft(){
        return controller.povLeft();
    }
    public Trigger getPovRight(){
        return controller.povRight();
    }
}
