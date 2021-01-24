/*----------------------------------------------------------------------------*/
/* Copyright (c) 2020 FIRST Team 2393. All Rights Reserved.                   */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.recharge.test;

import edu.wpi.first.wpilibj.Relay;
import edu.wpi.first.wpilibj.Relay.Value;
import frc.robot.BasicRobot;

/** Robot code for testing light */
public class LightTestRobot extends BasicRobot
{
  private final Relay light = new Relay(0);

  @Override
  public void teleopInit()
  {
    super.teleopInit();
    light.set(Value.kForward);
  }

  @Override
  public void disabledInit()
  {
    super.disabledInit();
    light.set(Value.kOff);
  }
}
