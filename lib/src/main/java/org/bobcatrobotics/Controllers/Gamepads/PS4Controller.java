package org.bobcatrobotics.Controllers.Gamepads;

import java.util.HashMap;
import java.util.Map;
import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class PS4Controller implements ControllerBase{
    private final edu.wpi.first.wpilibj.PS5Controller controller;
    private Map<String , BooleanSupplier> buttonMap = new HashMap<>();
    private final Alert controllerUnpluggedAlert;
    public PS4Controller(int port, String name){
        controllerUnpluggedAlert =
        new Alert(name + " Joystick unplugged!", AlertType.kWarning);
        controller = new edu.wpi.first.wpilibj.PS5Controller(port);
        buttonMap = Map.ofEntries(
            Map.entry("Square",()->controller.getSquareButton()),
            Map.entry("Cross",()->controller.getCrossButton()),
            Map.entry("Circle",()->controller.getCircleButton()),
            Map.entry("Triangle",()->controller.getTriangleButton())
            // Map.entry("L1",5),
            // Map.entry("R1",6),
            // Map.entry("L2",7),
            // Map.entry("R2",8),
            // Map.entry("Share",9),
            // Map.entry("Options",10),
            // Map.entry("L3",11),
            // Map.entry("R3",12),
            // Map.entry("PS",13),
            // Map.entry("Touchpad",14)
        );
    }
    public Trigger getButton(String buttonName){
        return new Trigger(buttonMap.get(buttonName));
    }
    public Trigger getLeftTrigger(){
        return new Trigger(()->controller.getL2Button());
    }
    public Trigger getRightTrigger(){
        return new Trigger(()->controller.getR2Button());
    }
    public Trigger getLeftBumper(){
        return new Trigger(()->controller.getL1Button());
    }
    public Trigger getRightBumper(){
        return new Trigger(()->controller.getR1Button());
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
        int pov = controller.getPOV();
        if(pov == 0){
            return new Trigger(()-> true);
        }
        return new Trigger(()-> false);
    }
    public Trigger getPovDown(){
        int pov = controller.getPOV();
        if(pov == 180){
            return new Trigger(()-> true);
        }
        return new Trigger(()-> false);
    }
    public Trigger getPovLeft(){
        int pov = controller.getPOV();
        if(pov == 90){
            return new Trigger(()-> true);
        }
        return new Trigger(()-> false);
    }
    public Trigger getPovRight(){
        int pov = controller.getPOV();
        if(pov == 270){
            return new Trigger(()-> true);
        }
        return new Trigger(()-> false);
    }
}
