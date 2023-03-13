// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autos;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Swerve;
import frc.robot.commands.MountChargeStation;
import frc.robot.commands.Autos.BalanceChargeStation;
import frc.robot.commands.SetToX;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class MountAndBalance extends SequentialCommandGroup {
  /** Creates a new MountAndBalance. */
  Swerve s_Swerve;

  public MountAndBalance(Swerve s_Swerve){
    this.s_Swerve = s_Swerve;
    addRequirements(s_Swerve);
    addCommands(
    new MountChargeStation(s_Swerve, false),
    new BalanceChargeStation(s_Swerve, true)
    
    );
  }
  
  
    
  }

