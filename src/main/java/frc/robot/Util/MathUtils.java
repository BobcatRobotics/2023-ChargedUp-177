// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Util;

/** Add your docs here. */
public class MathUtils{
    /**
     * modified sigmoid activation function
     * high values return 1, low values return -1
     */
    public static double throttlePercent(double val){
        return ((Math.exp(val)-1)/(Math.exp(val)+1));
    }
}
