package org.bobcatrobotics.Controllers.Gamepads;

import java.util.HashMap;
import java.util.Map;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class GenericHIDController implements ControllerBase {
    private final GenericHID controller;
    private final Alert controllerUnpluggedAlert;
    private Map<String , Integer> buttonMap = new HashMap<>();
    public GenericHIDController(int port, String name) {
        controllerUnpluggedAlert = new Alert(name + " Joystick unplugged!", AlertType.kWarning);
        controller = new GenericHID(port);
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
        return new Trigger(()->controller.getRawButton(buttonMap.get(buttonName).intValue()));
    }

    public Trigger getLeftTrigger() {
        return new Trigger(() -> controller.getRawButton(0));
    }

    public Trigger getRightTrigger() {
        return new Trigger(() -> controller.getRawButton(0));
    }

    public Trigger getLeftBumper() {
        return new Trigger(() -> controller.getRawButton(0));
    }

    public Trigger getRightBumper() {
        return new Trigger(() -> controller.getRawButton(0));
    }

    public double getLeftX() {
        return controller.getRawAxis(0);
    }

    public double getRightX() {
        return controller.getRawAxis(0);
    }

    public double getLeftY() {
        return controller.getRawAxis(0);
    }

    public double getRightY() {
        return controller.getRawAxis(0);
    }

    public void updateControllerAlerts() {
        controllerUnpluggedAlert.set(!controller.isConnected());
    }

    public Trigger getPovUp() {
        int pov = controller.getPOV();
        if (pov == 0) {
            return new Trigger(() -> true);
        }
        return new Trigger(() -> false);
    }

    public Trigger getPovDown() {
        int pov = controller.getPOV();
        if (pov == 180) {
            return new Trigger(() -> true);
        }
        return new Trigger(() -> false);
    }

    public Trigger getPovLeft() {
        int pov = controller.getPOV();
        if (pov == 90) {
            return new Trigger(() -> true);
        }
        return new Trigger(() -> false);
    }

    public Trigger getPovRight() {
        int pov = controller.getPOV();
        if (pov == 270) {
            return new Trigger(() -> true);
        }
        return new Trigger(() -> false);
    }
}
