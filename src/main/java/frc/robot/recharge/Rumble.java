/*----------------------------------------------------------------------------*/
/* Copyright (c) 2020 FIRST Team 2393. All Rights Reserved.                   */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.recharge;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.CommandBase;

/** Joystick Rumble */
public class Rumble extends CommandBase
{  
  private final Timer timer = new Timer();
  private double time = 0.1;

  /** Start to 'rumble' joystick for given seconds */
  public void schedule(final double seconds)
  {
    time = seconds;
    schedule();
  }

  @Override
  public void initialize()
  {
    timer.reset();
    timer.start();
    OI.joystick.setRumble(RumbleType.kLeftRumble, 1);
    OI.joystick.setRumble(RumbleType.kRightRumble, 1);
  }

  @Override
  public boolean isFinished()
  {
    return timer.hasElapsed(time);
  }

  @Override
  public void end(boolean interrupted)
  {
    OI.joystick.setRumble(RumbleType.kLeftRumble, 0);
    OI.joystick.setRumble(RumbleType.kRightRumble, 0);
  }
}
