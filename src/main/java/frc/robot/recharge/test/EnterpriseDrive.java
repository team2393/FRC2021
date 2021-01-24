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
public class EnterpriseDrive extends BasicRobot
{ 
  private final DriveTrain drive_train = new DriveTrain();

  // Commands that require the drive train, i.e. starting any of these commands
  // will cancel whatever else was running and required the drive train
  private final CommandBase reset_drivetrain = new Reset(drive_train);
  private final CommandBase drive_by_joystick = new DriveByJoystick(drive_train);
  private final HeadingHold heading_hold = new HeadingHold(drive_train);
  /** Most recent drive mode, either drive_by_joystick or heading_hold */
  private CommandBase drive_mode = heading_hold;

  private final Rumble rumble = new Rumble();
  
  // Shift commands can run concurrently with other commands that require the
  // drive train
  private final CommandBase shift_low = new InstantCommand(() -> drive_train.setGear(false));
  private final CommandBase shift_high = new InstantCommand(() -> drive_train.setGear(true));
  private final CommandBase auto_shift = new AutoShift(drive_train);

  @Override
  public void robotInit()
  {
    super.robotInit();

    // pcm.clearAllPCMStickyFaults();

    // Place some commands on dashboard
    SmartDashboard.putData("Reset Drive", reset_drivetrain);
    SmartDashboard.putData("Auto Shift", auto_shift);
  }

  @Override
  public void robotPeriodic()
  {
    super.robotPeriodic();
  }
  
  @Override
  public void teleopInit()
  {
    super.teleopInit();
    OI.reset();
    auto_shift.schedule();
    drive_mode.schedule();
  }
  
  @Override
  public void teleopPeriodic()
  {
    // Control hood angle via manual entry on dashboard or ApplySettings()
    teleop_drive();
  }

  private void teleop_drive()
  {
    
    if (! auto_shift.isScheduled())
    {
      // Manual shifting
      if (OI.isLowGearRequested())
        shift_low.schedule();
      else if (OI.isHighGearRequested())
        shift_high.schedule();
    }
    
    // Toggle between drive_by_joystick and heading_hold
    OI.force_low_speed = false;
    drive_mode.schedule();
    
 }

  @Override
  public void autonomousInit()
  {
    super.autonomousInit();

    OI.reset();
    drive_train.reset();
   }
}
