/*----------------------------------------------------------------------------*/
/* Copyright (c) 2020 FIRST Team 2393. All Rights Reserved.                   */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.recharge.drivetrain;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.CommandBase;

/** Control heading via PID */
public class TurnToHeading extends CommandBase 
{
  private final DriveTrain drive_train;
  // Limit profile to 90 deg/sec rotational speed
  private final ProfiledPIDController pid =
         new ProfiledPIDController(0.025, 0.03, 0,
                                   new TrapezoidProfile.Constraints(90.0, 45.0));
  private double kV =  0.0055;
  private final Timer timer = new Timer();

  public TurnToHeading(final DriveTrain drive_train, final double heading) 
  {
    this.drive_train = drive_train;
    // Good enough: Within 1 degree, slower than 1 deg/sec
    pid.setTolerance(1.0, 1.0);
    pid.setGoal(heading);
    addRequirements(drive_train);
  }

  /** @param degrees New desired heading */
  public void setDesiredHeading(final double degrees)
  {
    pid.reset(drive_train.getHeadingDegrees());
    pid.setGoal(degrees);
  }

  /** Change settings
   * 
   *  @param kV Feed forward gain, Volts per speed
   *  @param P Proportional
   *  @param I Integral
   *  @param D Differential gain
   */
  public void configure(final double kV, final double P, final double I, final double D)
  {
    this.kV = kV;
    pid.setP(P);
    boolean reset = false;
    if (I != pid.getI())
    {
      pid.setI(I);
      reset = true;
    }
    if (D != pid.getD())
    {
      pid.setD(D);
      reset = true;
    }
    // I or D changes require reset
    if (reset)
      pid.reset(drive_train.getHeadingDegrees());
  }

  @Override
  public void initialize()
  {
    pid.reset(drive_train.getHeadingDegrees());
    timer.reset();
    timer.start();
  }

  @Override
  public void execute()
  {
    final double correction = pid.calculate(drive_train.getHeadingDegrees());
    final double turn_speed = pid.getSetpoint().velocity;
    System.out.format("Turn to %.2f, current point %.2f at %.2f\n",
                      pid.getGoal().position,
                      pid.getSetpoint().position,
                      pid.getSetpoint().velocity);

    final double rotation = -turn_speed * kV - correction;

    drive_train.drive(0, rotation); 
  }

  @Override
  public boolean isFinished()
  {
    if (timer.hasElapsed(5.0))
    {
      System.err.println("TurnToHeading gives up (timeout)");
      return true;
    }
    return pid.atGoal();
  }
  
  @Override
  public void end(boolean interrupted)
  {
    drive_train.drive(0, 0);
  }
}
