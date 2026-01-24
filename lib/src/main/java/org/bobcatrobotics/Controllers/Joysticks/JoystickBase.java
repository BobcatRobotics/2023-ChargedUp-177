package org.bobcatrobotics.Controllers.Joysticks;

import edu.wpi.first.wpilibj2.command.button.Trigger;

public interface JoystickBase {
    Trigger getRawButton(String buttonName);
    double getStickX();
    double getStickY();
    double getStickZ();
    double getFineStickX();
    double getFineStickY();
    double getFineStickZ();
    void updateControllerAlerts();
}