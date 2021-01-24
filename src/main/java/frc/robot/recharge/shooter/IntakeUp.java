/*----------------------------------------------------------------------------*/
/* Copyright (c) 2020 FIRST Team 2393. All Rights Reserved.                   */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.recharge.shooter;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class IntakeUp extends CommandBase
{
  private final Intake intake;

  public IntakeUp(Intake intake) 
  {
    this.intake = intake;
    addRequirements(intake);
  }

  @Override
  public void initialize() 
  {
    intake.enableSpinner(false);
    intake.setIntakeAngle(65);
  }

  @Override
  public boolean isFinished()
  {
    return true;
  }
}
