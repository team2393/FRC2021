/*----------------------------------------------------------------------------*/
/* Copyright (c) 2020 FIRST Team 2393. All Rights Reserved.                   */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.recharge.test;

import frc.robot.BasicRobot;
import frc.robot.recharge.drivetrain.DriveTrain;
import frc.robot.recharge.drivetrain.RotateToTarget;

/** Robot code for testing camera and rotate-to-target */
public class RotateToTargetTestRobot extends BasicRobot
{
  // Real robot uses drivetrain, demo setup uses servo
  private final DriveTrain drive_train = new DriveTrain();

  private final RotateToTarget rotate_to_target = new RotateToTarget(drive_train);
  
  @Override
  public void robotInit()
  {
    super.robotInit();
  }

  @Override
  public void autonomousPeriodic()
  {
    rotate_to_target.schedule();
  }
}
