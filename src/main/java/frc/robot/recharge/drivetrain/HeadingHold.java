/*----------------------------------------------------------------------------*/
/* Copyright (c) 2020 FIRST Team 2393. All Rights Reserved.                   */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.recharge.drivetrain;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.recharge.OI;

/** Control heading via PID */
public class HeadingHold extends CommandBase 
{
  private final DriveTrain drive_train;
  
  /** Time (ms) when we may resume heading hold after steering */
  private long resume_heading_hold = 0;

  public HeadingHold(final DriveTrain drive_train) 
  {
    this.drive_train = drive_train;
    addRequirements(drive_train);    
  }

  public void setDesiredHeading(final double degrees)
  {
    drive_train.getHeadingPID().setSetpoint(degrees);
  }

  @Override
  public void initialize()
  {
    setDesiredHeading(drive_train.getHeadingDegrees());
  }

  @Override
  public void execute()
  {
    final long now = System.currentTimeMillis();

    // Moving stick enough to manually steer?
    final boolean steering = Math.abs(OI.getDirection()) >  0.07;
    if (steering)
    {
      drive_train.drive(OI.getSpeed(), OI.getDirection());
      setDesiredHeading(drive_train.getHeadingDegrees());
      resume_heading_hold = now + 1000;
    }
    else if (now < resume_heading_hold)
    {
      /*
      It seems that even though the left stick is released the robot 
      usually has a little inertia left and continues to spin and then the PID attempts
      to return to the position at the exact time the stick was released...
      ==> Until 'resume_heading_hold', use rotation=0
          and update PID to the current heading
      */
      drive_train.drive(OI.getSpeed(), 0);
      setDesiredHeading(drive_train.getHeadingDegrees());
    }
    else
    {
      // Not moving stick, and waited until 'resume_heading_hold'
      // ==> Use PID to hold heading
      final double rotation = drive_train.getHeadingPID().calculate(drive_train.getHeadingDegrees());
      drive_train.drive(OI.getSpeed(), -rotation);
    }
  }
  
  @Override
  public void end(boolean interrupted)
  {
    drive_train.drive(0, 0);
  }
}
