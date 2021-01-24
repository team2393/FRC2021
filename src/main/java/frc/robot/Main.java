/*----------------------------------------------------------------------------*/
/* Copyright (c) 2020 FIRST Team 2393. All Rights Reserved.                   */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.RobotBase;
import frc.robot.recharge.test.RotateToTargetTestRobot;

/** 'main' class, selects which robot to run */
public final class Main
{
  public static void main(String... args)
  {
    RobotBase.startRobot(RotateToTargetTestRobot::new);
  } 
}
