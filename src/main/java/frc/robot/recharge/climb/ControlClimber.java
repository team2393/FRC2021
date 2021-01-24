/*----------------------------------------------------------------------------*/
/* Copyright (c) 2020 FIRST Team 2393. All Rights Reserved.                   */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.recharge.climb;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.recharge.OI;

/** Control Telescope and Climber motor */
public class ControlClimber extends CommandBase
{
  private final Climber climber;

  public ControlClimber(final Climber climber)
  {
    this.climber= climber;
    addRequirements(climber);
  }

  @Override
  public void execute()
  {
    // Slow down
    double speed = OI.getTelescopeSpeed() * 0.25;
    double height = climber.getHeight();
    // Only allow going 'up' when below max,
    // or 'down' when above 0
    if ( (speed > 0  &&  height < Climber.max_height)  ||
         (speed < 0  &&  height > 0))
        climber.moveTelescope(speed);
    else
      climber.moveTelescope(0);
    
    climber.pullUp(OI.getClimbSpeed());
  }
}
