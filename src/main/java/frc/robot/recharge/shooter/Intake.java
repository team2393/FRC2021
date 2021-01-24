/*----------------------------------------------------------------------------*/
/* Copyright (c) 2020 FIRST Team 2393. All Rights Reserved.                   */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.recharge.shooter;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.wpilibj.controller.ArmFeedforward;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpiutil.math.MathUtil;
import frc.robot.recharge.RobotMap;

/** Power cell handling: Intake
 * 
 *  Intake picks 'fuel cell' balls from floor into hopper.
 */
public class Intake extends SubsystemBase 
{
  // Motors
  private final WPI_VictorSPX spinner = new WPI_VictorSPX(RobotMap.INTAKE_SPINNER);
  
  // Main rotator has encoder (angle)
  private final WPI_TalonSRX rotator = new WPI_TalonSRX(RobotMap.INTAKE_ROTATOR);
  private final WPI_TalonSRX rotator_slave = new WPI_TalonSRX(RobotMap.INTAKE_ROTATOR_SLAVE);

  // FF & PID
  // https://trickingrockstothink.com/blog_posts/2019/10/26/controls_supp_arm.html
  private ArmFeedforward angle_ff = new ArmFeedforward(0.0, 1.0, 0.0);
  private final PIDController angle_pid = new PIDController(0.3, 0, 0);

  private boolean run_spinner = false;
  private boolean unjam = false;

  /** Desired arm/rotator angle. Negative to disable PID */
  private double desired_angle = -1;

  public Intake()
  {
    PowerCellAccelerator.commonSettings(spinner, NeutralMode.Coast);
    // Spin up/down with delay to please the battery
    // spinner.configOpenloopRamp(0.6);

    PowerCellAccelerator.commonSettings(rotator, NeutralMode.Brake);
    PowerCellAccelerator.commonSettings(rotator_slave, NeutralMode.Brake);
    rotator.setInverted(false);
    // Encoder for position (angle)
    rotator.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute);    

    // Second motor follows the one that we control
    rotator_slave.setInverted(false);
    rotator_slave.follow(rotator);

    resetToStartPosition();
  }

  /** Offset to get 0 degree == horizontal */
  private double offset = 18.5;

  /** In theory, the absolute encoder knows the exact angle.
   *  We only need to correct for a fixed offset angle depending
   *  on how the encoder magnet is mounted in the encoder.
   * 
   *  In reality, we've seen that offset change.
   *  Manually moving the intake all the way up against the
   *  robot frame and then calling this will re-initialize
   *  the offset.
   */
  public void resetToStartPosition()
  {
    // Get angle reading without offset
    offset = 0;
    final double test = getAngle();
    // If the arm was all out, we'd like to get "0 degrees",
    // i.e. the offset would be -test.
    // Since the arm is all up, we'd like to get "75 degrees":
    offset = 75 - test;
    System.out.format("Intake arm offset: %.1f degrees\n", offset);
  }

  /** @return Rotator arm angle, degrees. 0 for horizontal, towards 90 for 'up' */
  public double getAngle()
  {    
    // An angle of zero (degrees/radians) must be 'horizontal'
    // because  ArmFeedforward  uses cos(angle) to determine impact of gravity,
    // which is at maximum for angle 0 (cos(0)=1) and vanishes at 90 deg (cos(90)=0)

    // Encoder provides 4096 ticks per 360 degrees
    final double encoder_angle = 360.0 / 4096; 
    // Gears & chain result in actual arm moving slower than the motor output
    final double gearing = 12.0 / 30.0;

    return offset - rotator.getSelectedSensorPosition() * encoder_angle * gearing;
  }

  /** @param speed Directly set rotator motor speed for testing */
  public void setRotatorMotor(final double speed)
  {
    // Disable automated control
    desired_angle = -1;
    rotator.set(speed);
  }

  /** @param kCos Cosine(angle) factor to compensate for gravity
   *  @param P Proportional gain for angle error
   *  @param D Differential gain for angle error
   */
  public void configure(final double kCos, final double P, final double D)
  {
    if (angle_ff.kcos != kCos)
      angle_ff = new ArmFeedforward(0, kCos, 0);
    
    if (P != angle_pid.getP()  ||  D != angle_pid.getD())
    {
      angle_pid.reset();
      angle_pid.setP(P);
      angle_pid.setD(D);
    }
  }

  /** @param angle Set angle for PID-controlled angle rotator, negative to disable */
  public void setIntakeAngle(final double angle)
  {
    if (desired_angle != angle)
      desired_angle = angle;
  }
  
  /** @param on Should intake spinner be on? */
  public void enableSpinner(final boolean on)
  {
    run_spinner = on;
  }
  
  @Override
  public void periodic()
  {
    if (unjam)
    {
      spinner.setVoltage(11);
    }
    else
    {
      // Spin intake rollers at good speed
      spinner.setVoltage(run_spinner ? -12.0 : 0.0);
    }
    
    if (desired_angle >= 0)
    {
      // If the desired angle is low (put arm out),
      // and the actual angle is as well,
      // simply turn motor off to let arm settle onto bumper.
      if (desired_angle < 10   &&  getAngle() < 10)
        rotator.setVoltage(0);
      else
      {
        final double correction = angle_pid.calculate(getAngle(), desired_angle);
        final double preset = angle_ff.calculate(Math.toRadians(desired_angle), 0);
        rotator.setVoltage(MathUtil.clamp((preset + correction), -3, 3));
      }
    }
  }

  public void unjam(boolean unjam) 
  {
    this.unjam = unjam;
  }
}
