/*----------------------------------------------------------------------------*/
/* Copyright (c) 2020 FIRST Team 2393. All Rights Reserved.                   */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/
package frc.robot.demo.motor;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.BasicRobot;

/** Example of PID control for position
 * 
 *  Uses the Falcon's encoder
 *  with PID control performed in the RoboRIO
 */
public class EncoderTestRobot extends BasicRobot
{
  private final WPI_TalonSRX motor = new WPI_TalonSRX(5);

  @Override
  public void robotInit()
  {
    super.robotInit();
    // Configure motor
    motor.configFactoryDefault();
    motor.setNeutralMode(NeutralMode.Coast);
    motor.configOpenloopRamp(0.5);

    
    // The standalong CTRE mag encoder supports absolute position,
    // but the built-in one always reports 0.
    // The default, integrated encoder gives 2048 ticks per revolution
    // motor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);

    // Absolute sensor returns 4096 ticks for 360 degrees.
    // While 'running', it will count up/down beyond full 360 degree turns,
    // but after a power cycle it only reports the angle within 0..360 based
    // on the detected magnetic field angle.
    motor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute);

    // So even if the absolute mode is supported,
    // for example with a TalonSRX controller connected to the mag encoder,
    // the absolute encoder only knows its exact position within one revolution.
    // For details see   
    // https://github.com/CrossTheRoadElec/Phoenix-Examples-Languages/blob/master/Java/MagEncoder_Absolute/src/main/java/frc/robot/Robot.java
  }

  @Override
  public void robotPeriodic()
  {
    super.robotPeriodic();
    final double deg_per_rev = 360.0 / 4096;
    SmartDashboard.putNumber("Revs", motor.getSelectedSensorPosition()*deg_per_rev);
    SmartDashboard.putNumber("RPS", motor.getSelectedSensorVelocity()*deg_per_rev*10.0);
  }

  @Override
  public void teleopInit()
  {
    super.teleopInit();

    // It's possible to reset the 'absolute' encoder to zero.
    // After a power cycle, however, it will again report the actual magnetic field angle,
    // not remembering an offset from a reset.
    motor.setSelectedSensorPosition(0);
  }
}
