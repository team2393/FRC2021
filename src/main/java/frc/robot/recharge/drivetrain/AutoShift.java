/*----------------------------------------------------------------------------*/
/* Copyright (c) 2020 FIRST Team 2393. All Rights Reserved.                   */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.recharge.drivetrain;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.recharge.OI;

/** Automatically shifts gear */
public class AutoShift extends CommandBase 
{
  private final DriveTrain drive_train;

  public AutoShift(DriveTrain drive_train) 
  {
    this.drive_train = drive_train;
    // Does not 'require' the drivetrain, can run with DriveByJoystick
  }

  @Override
  public void execute()
  {
    // Shift down when joystick near 'idle'
    if (Math.abs(OI.getSpeed()) < 0.1)
      drive_train.setGear(false);

    // Automatically shift into fast gear
    // if we are in low gear,
    // joystick pedal to the metal
    // and encoder indicates we reached the max. speed
    // that we can get in low gear
    if (!drive_train.isHighSpeed() &&
        Math.abs(OI.getSpeed()) > 0.9 &&
        Math.abs(drive_train.getSpeedMetersPerSecond()) > 2.5)
        drive_train.setGear(true);
  }
}
