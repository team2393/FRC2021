/*----------------------------------------------------------------------------*/
/* Copyright (c) 2020 FIRST Team 2393. All Rights Reserved.                   */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.recharge.test;

import frc.robot.BasicRobot;
import frc.robot.recharge.climb.Climber;
import frc.robot.recharge.climb.ControlClimber;

/** Robot code for testing climb */
public class ClimbTestRobot extends BasicRobot
{
  private final Climber climber = new Climber();
  private final ControlClimber control_climber = new ControlClimber(climber);

  @Override
  public void teleopInit()
  {
    control_climber.schedule();
  }

  @Override
  public void teleopPeriodic() 
  {
    super.teleopPeriodic();
    System.out.println("Height " + climber.getHeight());
  }
  // 1) Check if telescope moves up and down
  // 2) Check if 'pulling up'is correct direction
}
