package org.bobcatrobotics.Controllers.Joysticks;

public class JoystickFactory {
    public static JoystickBase logitech(int port,String name) { return new LogitechJoystick(port,name); }
    public static JoystickBase ruffy(int port,String name) { return new RuffyJoystick(port,name); }
}
