/*----------------------------------------------------------------------------*/
/* Copyright (c) 2020 FIRST Team 2393. All Rights Reserved.                   */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.recharge.drivetrain;

import java.util.function.BiConsumer;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpiutil.math.MathUtil;
import frc.robot.recharge.udp.CameraData;
import frc.robot.recharge.udp.CameraLight;
import frc.robot.recharge.udp.GuessingUDPReceiverThread;

/** Rotate to target based on camera info */
public class RotateToTarget extends CommandBase 
{
  /** Relay for turning camera light on/off.
   *  Since only this command uses the light,
   *  create it with the first instance.
   */
  private static CameraLight light = null;
  
  private final DriveTrain drive_train;

  private final Timer timeout_timer = new Timer();
  
  public GuessingUDPReceiverThread udp;
  
  private boolean on_target;
  
  public RotateToTarget(final DriveTrain drive_train)
  {
    if (light == null)
      light = new CameraLight();
    
    try
    {
      udp = new GuessingUDPReceiverThread(5801);
    }
    catch (Exception ex)
    {
      throw new RuntimeException(ex);
    }

    this.drive_train = drive_train;
    addRequirements(drive_train);

    SmartDashboard.setDefaultNumber("TargetRotMin", 0.08);
    SmartDashboard.setDefaultNumber("TargetRotGain", 0.00325);
    SmartDashboard.setDefaultNumber("TargetRotMax", 0.500);
    SmartDashboard.setDefaultNumber("Desired Direction", 0);
    SmartDashboard.setDefaultNumber("Desired Distance", -500);
  }

  @Override
  public void initialize()
  {
    on_target = false;
    timeout_timer.reset();
    timeout_timer.start();
    light.set(true);
    System.out.println("RotateToTarget");
  }

  @Override
  public void execute()
  {
    final CameraData guess = udp.get();
    
    double direction_error = 0;
    double rotation = 0;

    // Don't react when direction is 0/unknown,
    // or when detected target is too far off to the side
    if (guess.direction != 0.0  &&  Math.abs(guess.direction) < 150)
    {
      direction_error = guess.direction - SmartDashboard.getNumber("Desired Direction", 0);

      // Proportial gain controller with some minimum
      rotation = Math.signum(direction_error) * SmartDashboard.getNumber("TargetRotMin", 0.0) + 
                 direction_error * SmartDashboard.getNumber("TargetRotGain", 0.0);
    }

    double desired_distance = SmartDashboard.getNumber("Desired Distance", -500); 
    double speed = 0;
    double position_error = 0;
    
    // Make sure desired distance is within screen
    if (Math.abs(desired_distance) < 120)
    {
      position_error = (guess.distance == 0)  ?  0  :  (desired_distance - guess.distance);
      speed = position_error * 10 * SmartDashboard.getNumber("TargetRotGain", 0.001000);
    }
    
    final double max = SmartDashboard.getNumber("TargetRotMax", 1.000);
    drive_train.drive(MathUtil.clamp(speed, -max, max), MathUtil.clamp(rotation, -max, max));
    
    if (Math.abs(direction_error) < 2 && Math.abs(position_error) < 2)
      on_target = true;
  }
  
  @Override
  public boolean isFinished()
  {
    if (timeout_timer.hasElapsed(5.0))
    {
      System.err.println("RotateToTarget gives up (timeout)");
      return true;
    }
    if (on_target)
      System.out.println("On target");
    return on_target;
  }

  @Override
  public void end(boolean interrupted)
  {
    drive_train.drive(0.0, 0.0);
    light.set(false);
  }
}
