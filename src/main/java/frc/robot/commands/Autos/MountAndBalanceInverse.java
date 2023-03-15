// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autos;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.MountChargeStationInverse;
import frc.robot.subsystems.Swerve;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class MountAndBalanceInverse extends SequentialCommandGroup {
  /** Creates a new MountAndBalanceInverse. */
  private Swerve s_Swerve;

  public MountAndBalanceInverse(Swerve s_Swerve) {
    this.s_Swerve = s_Swerve;
    addRequirements(s_Swerve);
    addCommands(
      new MountChargeStationInverse(s_Swerve),
      new BalanceChargeStationInverse(s_Swerve, true)
    );
  }
}
