/*----------------------------------------------------------------------------*/
/* Copyright (c) 2020 FIRST Team 2393. All Rights Reserved.                   */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.recharge.ctrlpanel;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.recharge.OI;

/** Command to turn wheel with joystick/button */
public class ManualWheelSpeed extends CommandBase
{
  private final ControlWheel wheel;

  public ManualWheelSpeed(final ControlWheel wheel)
  {
    this.wheel = wheel;
    addRequirements(wheel);
  }

  @Override
  public void execute()
  {
    wheel.spin(OI.getWheelSpeed());
  }

  @Override
  public void end(final boolean interrupted)
  {
    wheel.spin(0);
  }
}
