/*----------------------------------------------------------------------------*/
/* Copyright (c) 2020 FIRST Team 2393. All Rights Reserved.                   */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.recharge.drivetrain;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.recharge.OI;

/** Manually control speed and rotation via joystick */
public class DriveByJoystick extends CommandBase 
{
  private final DriveTrain drive_train;

  public DriveByJoystick(DriveTrain drive_train) 
  {
    this.drive_train = drive_train;
    addRequirements(drive_train);
  }

  @Override
  public void execute()
  {
    // // Test speed-based control (feed forward & PID):
    // // Use speed stick to request speed
    // double speed = OI.getSpeed()*2.5;
    // SmartDashboard.putNumber("Desired Speed", speed);

    // // Tweak a little with rotation stick to allow turning
    // drive_train.driveSpeed(speed + OI.getDirection(),
    //                        speed - OI.getDirection());

    // Normal joystick usage
    drive_train.drive(OI.getSpeed(), OI.getDirection());
  }

  @Override
  public void end(final boolean interrupted)
  {
    drive_train.drive(0, 0);
  }
}
