/*----------------------------------------------------------------------------*/
/* Copyright (c) 2020 FIRST Team 2393. All Rights Reserved.                   */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.recharge.test;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.BasicRobot;
import frc.robot.recharge.OI;
import frc.robot.recharge.Rumble;
import frc.robot.recharge.drivetrain.AutoShift;
import frc.robot.recharge.drivetrain.DriveByJoystick;
import frc.robot.recharge.drivetrain.DriveTrain;
import frc.robot.recharge.drivetrain.HeadingHold;
import frc.robot.recharge.drivetrain.Reset;
/**
 * Robot for 'Infinite Recharge' - R!$E2geTHeR#2020
 */
public class DriveSpeedTest extends BasicRobot
{ 
  private final DriveTrain drive_train = new DriveTrain();

  // Commands that require the drive train, i.e. starting any of these commands
  // will cancel whatever else was running and required the drive train
  private final CommandBase reset_drivetrain = new Reset(drive_train);
  /** Most recent drive mode, either drive_by_joystick or heading_hold */
  
  @Override
  public void robotInit()
  {
    super.robotInit();
    
    SmartDashboard.setDefaultNumber("P Value", 0.0);

    // pcm.clearAllPCMStickyFaults();

    // Place some commands on dashboard
    SmartDashboard.putData("Reset Drive", reset_drivetrain);
  }

  @Override
  public void robotPeriodic()
  {
    super.robotPeriodic();
  }
  
  @Override
  public void teleopInit()
  {
    drive_train.lock(true);
    super.teleopInit();
    OI.reset();
    drive_train.setGear(false);
  }
  
  @Override
  public void disabledInit() 
  {
    drive_train.lock(false);
  }

  @Override
  public void teleopPeriodic()
  {
    // Control hood angle via manual entry on dashboard or ApplySettings()
    double requested_speed = OI.getSpeed() * 3;
    drive_train.driveSpeed(requested_speed, requested_speed);
    double actual_speed = drive_train.getSpeedMetersPerSecond();

    double speed_error = requested_speed - actual_speed;

    SmartDashboard.putNumber("Actual Speed", actual_speed);
    SmartDashboard.putNumber("Requested Speed", requested_speed);
    SmartDashboard.putNumber("Speed Error", speed_error);

    double p_value = SmartDashboard.getNumber("P Value", 0.0);
    drive_train.configureP(p_value);
  }
}
