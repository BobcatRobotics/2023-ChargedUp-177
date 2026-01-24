package org.bobcatrobotics.Controllers.Gamepads;

import java.util.HashMap;
import java.util.Map;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class XboxController  implements ControllerBase{
    private final CommandXboxController controller;
    private Map<String , Integer> buttonMap = new HashMap<>();
    private final Alert controllerUnpluggedAlert;
    public XboxController(int port, String name){
        controllerUnpluggedAlert =
        new Alert(name + " Joystick unplugged!", AlertType.kWarning);
        controller = new CommandXboxController(port);
        buttonMap = Map.ofEntries(
            Map.entry("A",1),
            Map.entry("B",2),
            Map.entry("X",3),
            Map.entry("Y",4),
            Map.entry("LeftBumper",5),
            Map.entry("RightBumper",6),
            Map.entry("Back",7),
            Map.entry("Start",8)
        );
    }
    public Trigger getButton( String buttonName ){
        return controller.button(buttonMap.get(buttonName).intValue());
    }
    public Trigger getLeftTrigger(){
        return controller.leftTrigger(0.1);
    }
    public Trigger getRightTrigger(){
        return controller.rightTrigger(0.1);
    }
    public Trigger getLeftBumper(){
        return controller.leftBumper();
    }
    public Trigger getRightBumper(){
        return controller.rightBumper();
    }
    public double getLeftX(){
        return controller.getLeftX();
    }
    public double getRightX(){
        return controller.getRightX();
    }
    public double getLeftY(){
        return controller.getLeftY();
    }
    public double getRightY(){
        return controller.getRightY();
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
