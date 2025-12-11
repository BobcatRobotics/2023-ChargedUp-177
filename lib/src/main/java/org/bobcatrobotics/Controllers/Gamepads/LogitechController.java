package org.bobcatrobotics.Controllers.Gamepads;

import java.util.HashMap;
import java.util.Map;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class LogitechController {
    CommandJoystick controller;
    private Map<String , Integer> buttonMap = new HashMap<>();
    private final Alert controllerUnpluggedAlert;
    public LogitechController(int port, String name){
        controllerUnpluggedAlert =
        new Alert(name + " Joystick unplugged!", AlertType.kWarning);
        controller = new CommandJoystick(port);
        buttonMap = Map.ofEntries(
            Map.entry("A",1),
            Map.entry("B",2),
            Map.entry("X",3),
            Map.entry("Y",4),
            Map.entry("Select",9),
            Map.entry("Start",10)
        );
    }

    public Trigger getButton(String buttonName) {
        return controller.button(buttonMap.get(buttonName).intValue());
    }
    public Trigger getLeftTrigger(){
        return new Trigger(()-> false);
    }
    public Trigger getRightTrigger(){
        return new Trigger(()-> false);
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
        return -controller.getRawAxis(5);
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
