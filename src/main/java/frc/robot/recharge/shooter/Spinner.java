/*----------------------------------------------------------------------------*/
/* Copyright (c) 2020 FIRST Team 2393. All Rights Reserved.                   */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.recharge.shooter;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.controller.PIDController;
import frc.robot.recharge.RobotMap;

/** RPM-controlled spinner */
public class Spinner
{
  private final WPI_TalonFX motor = new WPI_TalonFX(RobotMap.SHOOTER_MOTOR);
  private final WPI_TalonFX follower = new WPI_TalonFX(RobotMap.SHOOTER_SLAVE_MOTOR);

  /** Encoder ticks for one turn of the wheel */
  private final static double TICK_PER_REVOLUTION = 3310 * 220/360;
  
  // FF & PID for shooter motor to set RPM
  // https://trickingrockstothink.com/blog_posts/2019/10/19/tuning_pid.html

  /** Feed-forward velocity constant: Volts per RPM
   *  Voltage: 11.444091796875 RPM: 5253.412462908012
   *  Voltage: 11.6279296875 RPM: 5696.142433234421
   */
  private double kV = 0.00188;
  private double k0 = 0.76;
  /** P gain */
  private final PIDController pid = new PIDController(0.0025, 0, 0);

  public Spinner()
  {
    motor.configFactoryDefault();
    motor.clearStickyFaults();
    motor.setNeutralMode(NeutralMode.Coast);
    motor.setInverted(true);
    motor.setSelectedSensorPosition(0);
    // motor.configOpenloopRamp(0.5);
    
    // Add second shooter motor
    follower.configFactoryDefault();
    follower.clearStickyFaults();
    follower.setNeutralMode(NeutralMode.Coast);
    follower.setInverted(true);
    follower.follow(motor);
  }  
  
  public void configure(double kV, double P)
  {
    this.kV = kV;
    pid.setP(P);
  }

  public void setVoltage(final double volt) 
  {
    motor.setVoltage(volt);
    // final double current = motor.getStatorCurrent();  
    // if (current > 250)
    // {
    //   motor.setVoltage(0);
    //   System.out.println("Overcurrent at " + motor.getStatorCurrent() + " stopping motor" );
    // }
  }  

  public double getAngle()
  {
    return motor.getSelectedSensorPosition() / TICK_PER_REVOLUTION * 360.0;
  }

  public double getRPM()
  {
    // Sensor gives ticks per 100ms, i.e. 10 times that per second.
    // Turn into revolutions, scale to revs per minute.
    return motor.getSelectedSensorVelocity() * 10.0 / TICK_PER_REVOLUTION * 60.0;
  }  

  public void setRPM(final double desired_rpm)
  {
    if (desired_rpm < 100)
      setVoltage(0);
    else
    {
      final double feed_forward = k0 + desired_rpm*kV; 
      final double voltage = feed_forward + pid.calculate(getRPM(), desired_rpm);
      setVoltage(voltage);
    }
  }
}
