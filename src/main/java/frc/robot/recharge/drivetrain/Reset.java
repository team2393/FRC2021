/*----------------------------------------------------------------------------*/
/* Copyright (c) 2020 FIRST Team 2393. All Rights Reserved.                   */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.recharge.drivetrain;

import edu.wpi.first.wpilibj2.command.InstantCommand;

/** Reset drivetrain */
public class Reset extends InstantCommand 
{
  public Reset(final DriveTrain drive_train) 
  {
    super(drive_train::reset, drive_train);
  }
}
