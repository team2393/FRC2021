/*----------------------------------------------------------------------------*/
/* Copyright (c) 2020 FIRST Team 2393. All Rights Reserved.                   */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.recharge.test;

import java.util.function.Supplier;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.BasicRobot;
import frc.robot.recharge.OI;

/** Based on the robot code created by the python
 *  'frc-characterization'
 *  for an 'arm'.
 */
public class CharacterizationRobotArm extends BasicRobot
{  
  // The offset of encoder zero from horizontal, in degrees.
  // It is CRUCIAL that this be set correctly, or the characterization will not
  // work!
  static private double OFFSET = 0;
  static private double ENCODER_EDGES_PER_REV = 4906;
  static private double encoderConstant = (1.0 / ENCODER_EDGES_PER_REV) * 360.;

  
  // Select motor, configure encoder
  final WPI_TalonFX motor = new WPI_TalonFX(1);
  final Supplier<Double> encoderPosition = () -> motor.getSelectedSensorPosition() * encoderConstant + OFFSET;
  final Supplier<Double> encoderRate = () -> motor.getSelectedSensorVelocity() * encoderConstant * 10.0;


  // frc-characterization commands robot via this entry
  final NetworkTableEntry autoSpeedEntry = NetworkTableInstance.getDefault().getEntry("/robot/autospeed");
  // Robot reports measurements to frc-characterization via this entry
  final NetworkTableEntry telemetryEntry = NetworkTableInstance.getDefault().getEntry("/robot/telemetry");

  final Number[] numberArray = new Number[6];
  double priorAutospeed = 0;

  @Override
  public void robotInit()
  {
    super.robotInit();

    motor.setNeutralMode(NeutralMode.Brake);
    motor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
    motor.setInverted(false);
    motor.setSensorPhase(false);

    // Reset encoders
    motor.setSelectedSensorPosition(0);

    // Set the update rate instead of using flush because of a ntcore bug
    // -> probably don't want to do this on a robot in competition
    NetworkTableInstance.getDefault().setUpdateRate(0.010);
  }
  
  @Override
  public void disabledInit()
  {
    super.disabledInit();
    motor.set(0);
  }
  
  @Override
  public void robotPeriodic()
  {
    // Not calling super.robotPeriodic() since not using commands

    // Feedback for users, but not used by the control program
    SmartDashboard.putNumber("encoder_pos", encoderPosition.get());
    SmartDashboard.putNumber("encoder_rate", encoderRate.get());
  }

  @Override
  public void teleopPeriodic()
  {
    motor.set(OI.getSpeed());
  }
  
  @Override
  public void autonomousPeriodic()
  {
    // Retrieve values to send back before telling the motors to do something
    double now = Timer.getFPGATimestamp();

    double position = encoderPosition.get();
    double rate = encoderRate.get();
    
    double battery = RobotController.getBatteryVoltage();
    double motorVolts = battery * Math.abs(priorAutospeed);
    
    // Retrieve the commanded speed from NetworkTables
    double autospeed = autoSpeedEntry.getDouble(0);
    priorAutospeed = autospeed;

    // command motors to do things
    motor.set(autospeed);

    // send telemetry data array back to NT
    numberArray[0] = now;
    numberArray[1] = battery;
    numberArray[2] = autospeed;
    numberArray[3] = motorVolts;
    numberArray[4] = position;
    numberArray[5] = rate;

    telemetryEntry.setNumberArray(numberArray);    
  }
}
