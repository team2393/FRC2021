/*----------------------------------------------------------------------------*/
/* Copyright (c) 2020 FIRST Team 2393. All Rights Reserved.                   */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.recharge.drivetrain;

import edu.wpi.first.wpilibj2.command.InstantCommand;

public class ToggleGearShift extends InstantCommand 
{
  private final DriveTrain drive_train;
  public ToggleGearShift(DriveTrain drive_train) 
  {
    this.drive_train = drive_train;
  }

  @Override
  public void initialize() 
  {
    drive_train.setGear(!drive_train.isHighSpeed());
  }
}
