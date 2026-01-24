package org.bobcatrobotics.Controllers.Gamepads;

public class ControllerFactory {
    public static ControllerBase xbox(int port,String name) { return new XboxController(port,name); }
    public static ControllerBase ps4(int port,String name) { return new PS4Controller(port,name); }
    public static ControllerBase ps5(int port,String name) { return new PS5Controller(port,name); }
    public static ControllerBase eightBitDo(int port,String name) { return new EightBitDoController(port,name); }
}
