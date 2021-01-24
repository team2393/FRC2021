/*----------------------------------------------------------------------------*/
/* Copyright (c) 2020 FIRST Team 2393. All Rights Reserved.                   */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.recharge.drivetrain;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpiutil.math.MathUtil;

/** Control position via PID */
public class DriveToPosition extends CommandBase 
{
  private final DriveTrain drive_train;

  public DriveToPosition(final DriveTrain drive_train) 
  {
    this.drive_train = drive_train;
    addRequirements(drive_train);
  }

  public void setDesiredPosition(final double meters)
  {
    drive_train.getPositionPID().setSetpoint(meters);
  }

  @Override
  public void initialize()
  {
    super.initialize();
    drive_train.setGear(false);
    drive_train.getPositionPID().reset();
  }

  @Override
  public void execute()
  {
    final double speed = drive_train.getPositionPID().calculate(drive_train.getPositionMeters());
    drive_train.drive(MathUtil.clamp(speed, -0.8, 0.8), 0);
  }

  @Override
  public boolean isFinished()
  {
    return drive_train.getPositionPID().atSetpoint();
  }

  @Override
  public void end(boolean interrupted)
  {
    drive_train.drive(0, 0);
  }
}
