package org.bobcatrobotics.Controllers.Gamepads;

import edu.wpi.first.wpilibj2.command.button.Trigger;

public interface ControllerBase {
    double getLeftX();
    double getLeftY();
    double getRightX();
    double getRightY();
    Trigger getButton( String buttonName );
    Trigger getLeftTrigger();
    Trigger getRightTrigger();
    Trigger getLeftBumper();
    Trigger getRightBumper();
    Trigger getPovUp();
    Trigger getPovDown();
    Trigger getPovLeft();
    Trigger getPovRight();
    void updateControllerAlerts();
}