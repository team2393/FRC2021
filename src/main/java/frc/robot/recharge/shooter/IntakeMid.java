/*----------------------------------------------------------------------------*/
/* Copyright (c) 2020 FIRST Team 2393. All Rights Reserved.                   */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.recharge.shooter;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class IntakeMid extends  CommandBase 
{
  private final Intake intake;

  public IntakeMid(Intake intake) 
  {
    this.intake = intake;
    addRequirements(intake);
  }

  @Override
  public void initialize() 
  {
    intake.enableSpinner(false);
    intake.setIntakeAngle(30);
  }

  @Override
  public boolean isFinished()
  {
    return true;
  }
}
