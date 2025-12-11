package org.bobcatrobotics.Controllers;

import org.bobcatrobotics.Controllers.Gamepads.ControllerBase;
import org.bobcatrobotics.Controllers.Gamepads.EightBitDoController;
import org.bobcatrobotics.Controllers.Gamepads.GenericHIDController;
import org.bobcatrobotics.Controllers.Gamepads.PS4Controller;
import org.bobcatrobotics.Controllers.Gamepads.PS5Controller;
import org.bobcatrobotics.Controllers.Gamepads.XboxController;
import org.bobcatrobotics.Controllers.Joysticks.GenericHIDJoystick;
import org.bobcatrobotics.Controllers.Joysticks.JoystickBase;
import org.bobcatrobotics.Controllers.Joysticks.LogitechJoystick;
import org.bobcatrobotics.Controllers.Joysticks.RuffyJoystick;
import edu.wpi.first.wpilibj.DriverStation;

public class ControllerAutoDetect {
    public static ControllerBase createGamepad(int port, String controllerName) {

        String name = DriverStation.getJoystickName(port).toLowerCase();
        boolean isXbox = DriverStation.getJoystickIsXbox(port);

        // Debug print (remove in production)
        System.out.println("[ControllerDetect] Port " + port + " -> " + name);

        // --------------------------
        // XBOX (most reliable test)
        // --------------------------
        if (isXbox || name.contains("xbox")) {
            System.out.println("[ControllerDetect] Detected XBOX controller.");
            return new XboxController(port, controllerName);
        }

        // --------------------------
        // PLAYSTATION 4
        // USB name examples:
        // - "Wireless Controller"
        // - "Sony Interactive Entertainment Wireless Controller"
        // --------------------------
        if (name.contains("ps4") || name.contains("sony") || name.contains("wireless controller")) {
            System.out.println("[ControllerDetect] Detected PS4 controller.");
            return new PS4Controller(port, controllerName);
        }

        // --------------------------
        // PLAYSTATION 5
        // USB name examples:
        // - "DualSense Wireless Controller"
        // - "PS5 Controller"
        // --------------------------
        if (name.contains("ps5") || name.contains("dualsense")) {
            System.out.println("[ControllerDetect] Detected PS5 controller.");
            return new PS5Controller(port, controllerName);
        }

        // --------------------------
        // 8BitDo (many modes: XInput, Switch, DInput)
        // USB examples include:
        // - "8bitdo"
        // - "pro 2"
        // - "sn30"
        // --------------------------
        if (name.contains("8bitdo") || name.contains("pro 2") || name.contains("sn30")) {
            System.out.println("[ControllerDetect] Detected 8BitDo controller.");
            return new EightBitDoController(port, controllerName);
        }

        // --------------------------
        // FALLBACK
        // --------------------------
        System.out
                .println("[ControllerDetect] Unknown controller type. Using GenericHIDController.");
        return new GenericHIDController(port, controllerName);
    }

    public static JoystickBase createJoystick(int port, String joystickName) {

        String name = DriverStation.getJoystickName(port).toLowerCase();

        // Debug print (remove in production)
        System.out.println("[ControllerDetect] Port " + port + " -> " + name);

        // --------------------------
        // ruffy
        // --------------------------
        if (name.contains("ruffy")) {
            System.out.println("[ControllerDetect] Detected ruffy joystick.");
            return new RuffyJoystick(port, joystickName);
        }

        // --------------------------
        // logitech
        // --------------------------
        if (name.contains("logitech")) {
            System.out.println("[ControllerDetect] Detected logitech joystick.");
            return new LogitechJoystick(port, joystickName);
        }


        // --------------------------
        // FALLBACK
        // --------------------------
        System.out
                .println("[ControllerDetect] Unknown controller type. Using GenericHIDController.");
        return new GenericHIDJoystick(port, joystickName);
    }
}
