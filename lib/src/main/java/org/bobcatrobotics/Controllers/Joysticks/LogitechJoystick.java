package org.bobcatrobotics.Controllers.Joysticks;

import java.util.HashMap;
import java.util.Map;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class LogitechJoystick implements JoystickBase{
    private CommandJoystick controller;
    private final Alert joystickUnpluggedAlert;
    private Map<String , Integer> buttonMap = new HashMap<>();

    public LogitechJoystick(int port, String name){
        joystickUnpluggedAlert =
      new Alert(name + " Joystick unplugged!", AlertType.kWarning);
        controller = new CommandJoystick(port);
        buttonMap = Map.ofEntries(
            Map.entry("A",1)
        );
    }
    public Trigger getRawButton( String buttonName ){
        return controller.button(buttonMap.get(buttonName).intValue());
    }
    public double getStickX(){
        return controller.getRawAxis(0);
    }
    public double getStickY(){
        return controller.getRawAxis(1);
    }
    public double getStickZ(){
        return controller.getRawAxis(2);
    }
    public double getFineStickX(){
        return controller.getRawAxis(0);
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
