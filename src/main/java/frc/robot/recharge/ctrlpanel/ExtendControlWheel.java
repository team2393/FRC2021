/*----------------------------------------------------------------------------*/
/* Copyright (c) 2020 FIRST Team 2393. All Rights Reserved.                   */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/
package frc.robot.recharge.ctrlpanel;

import edu.wpi.first.wpilibj2.command.InstantCommand;

/** Command to extend the control wheel assembly */
public class ExtendControlWheel extends InstantCommand
{
  public ExtendControlWheel(final ControlWheel wheel)
  {
    super(() -> wheel.extend(true));
  }
}
