/*----------------------------------------------------------------------------*/
/* Copyright (c) 2020 FIRST Team 2393. All Rights Reserved.                   */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.recharge.climb;

import edu.wpi.first.wpilibj2.command.CommandBase;

/** Turn conveyors etc. off */
public class ClimbIdle extends CommandBase
{
  private final Climber climber;

  public ClimbIdle(final Climber climber)
  {
    this.climber= climber;
    addRequirements(climber);
  }

  @Override
  public void execute()
  {
    // As long as they are not being used, set both motors to 0
    climber.moveTelescope(0);
    climber.pullUp(0);
  }
}
