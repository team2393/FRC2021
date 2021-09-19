/*----------------------------------------------------------------------------*/
/* Copyright (c) 2020 FIRST Team 2393. All Rights Reserved.                   */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.recharge.test;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj.Relay;
import edu.wpi.first.wpilibj.Relay.Value;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.BasicRobot;

/** Robot code for testing light */
public class LimitSwitchTestRobot extends BasicRobot
{
  DigitalInput top_limit_switch = new DigitalInput(4);
  @Override
  public void teleopPeriodic() {
    boolean limit_triggered = top_limit_switch.get();
    SmartDashboard.putBoolean("Top Limit", limit_triggered);
  }

}