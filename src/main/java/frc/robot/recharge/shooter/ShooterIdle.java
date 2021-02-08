/*----------------------------------------------------------------------------*/
/* Copyright (c) 2020 FIRST Team 2393. All Rights Reserved.                   */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.recharge.shooter;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.recharge.shooter.PowerCellAccelerator.State;

/** Turn conveyors etc. off */
public class ShooterIdle extends InstantCommand
{
  public ShooterIdle(final PowerCellAccelerator pca)
  {
    super(() ->
    {
      pca.setState(State.OFF);
    }, pca);
  }
}
