/*----------------------------------------------------------------------------*/
/* Copyright (c) 2020 FIRST Team 2393. All Rights Reserved.                   */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.recharge.test;

import java.util.function.Supplier;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.BasicRobot;
import frc.robot.recharge.OI;
import frc.robot.recharge.drivetrain.DriveTrain;

/** Based on the robot code created by the python
 *  'frc-characterization drive new'
 *  command, adapted to use our drive train settings.
 */
public class CharacterizationRobot extends BasicRobot
{  
  final DriveTrain drive_train = new DriveTrain();

  final Supplier<Double> leftEncoderPosition = drive_train::getLeftPositionMeters;
  final Supplier<Double> leftEncoderRate = drive_train::getLeftSpeedMetersPerSecond;
  final Supplier<Double> rightEncoderPosition = drive_train::getRightPositionMeters;
  final Supplier<Double> rightEncoderRate = drive_train::getRightSpeedMetersPerSecond;
  final Supplier<Double> gyroAngleRadians = () -> Math.toRadians(drive_train.getHeadingDegrees());

  // frc-characterization commands robot speed and rotation via these entries
  final NetworkTableEntry autoSpeedEntry = NetworkTableInstance.getDefault().getEntry("/robot/autospeed");
  final NetworkTableEntry rotateEntry = NetworkTableInstance.getDefault().getEntry("/robot/rotate");
  // Robot reports measurements to frc-characterization via this entry
  final NetworkTableEntry telemetryEntry = NetworkTableInstance.getDefault().getEntry("/robot/telemetry");

  final Number[] numberArray = new Number[10];
  double priorAutospeed = 0;

  @Override
  public void robotInit()
  {
    super.robotInit();

    // Set the update rate instead of using flush because of a ntcore bug
    // -> probably don't want to do this on a robot in competition
    NetworkTableInstance.getDefault().setUpdateRate(0.010);
  }
  
  @Override
  public void disabledInit()
  {
    super.disabledInit();
    drive_train.drive(0, 0);
  }
  
  @Override
  public void robotPeriodic()
  {
    // Not calling super.robotPeriodic() since not using commands

    // Feedback for users, but not used by the control program
    SmartDashboard.putNumber("l_encoder_pos", leftEncoderPosition.get());
    SmartDashboard.putNumber("l_encoder_rate", leftEncoderRate.get());
    SmartDashboard.putNumber("r_encoder_pos", rightEncoderPosition.get());
    SmartDashboard.putNumber("r_encoder_rate", rightEncoderRate.get());
  }

  @Override
  public void teleopPeriodic()
  {
    drive_train.drive(OI.getSpeed(), OI.getDirection());
  }
  
  @Override
  public void autonomousPeriodic()
  {
    // Retrieve values to send back before telling the motors to do something
    double now = Timer.getFPGATimestamp();

    double leftPosition = leftEncoderPosition.get();
    double leftRate = leftEncoderRate.get();

    double rightPosition = rightEncoderPosition.get();
    double rightRate = rightEncoderRate.get();

    double battery = RobotController.getBatteryVoltage();
    double motorVolts = battery * Math.abs(priorAutospeed);

    double leftMotorVolts = motorVolts;
    double rightMotorVolts = motorVolts;

    // Retrieve the commanded speed from NetworkTables
    double autospeed = autoSpeedEntry.getDouble(0);
    priorAutospeed = autospeed;

    // command motors to do things
    drive_train.tankDrive((rotateEntry.getBoolean(false) ? -1 : 1) * autospeed,
                          autospeed);

    // send telemetry data array back to NT
    numberArray[0] = now;
    numberArray[1] = battery;
    numberArray[2] = autospeed;
    numberArray[3] = leftMotorVolts;
    numberArray[4] = rightMotorVolts;
    numberArray[5] = leftPosition;
    numberArray[6] = rightPosition;
    numberArray[7] = leftRate;
    numberArray[8] = rightRate;
    numberArray[9] = gyroAngleRadians.get();

    telemetryEntry.setNumberArray(numberArray);    
  }
}
