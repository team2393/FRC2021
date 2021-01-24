/*----------------------------------------------------------------------------*/
/* Copyright (c) 2020 FIRST Team 2393. All Rights Reserved.                   */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/
package frc.robot.demo.motor;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpiutil.math.MathUtil;
import frc.robot.BasicRobot;

/** Example of PID control for position
 * 
 *  Uses the Falcon's encoder
 *  with PID control performed in the RoboRIO
 */
public class FalconTestRobot extends BasicRobot
{
  private final WPI_TalonFX motor = new WPI_TalonFX(1);

  private static final double steps_per_rev = 2048;

  private final PIDController pid = new PIDController(0.15, 0, 0.0002);

  private double request_per_rps = 0.25 / 24.5;
  private final TrapezoidProfile.Constraints constraints = new TrapezoidProfile.Constraints(10.0, 10.0);
  private final ProfiledPIDController profiled_pid = new ProfiledPIDController(0.15, 0, 0, constraints);
  
  public double getRevs()
  {
    return motor.getSelectedSensorPosition()/steps_per_rev;
  }

  public double getRPS()
  {
    return motor.getSelectedSensorVelocity()/steps_per_rev * 10.0;
  }

  @Override
  public void robotInit()
  {
    super.robotInit();
    // Configure motor
    motor.configFactoryDefault();
    motor.setNeutralMode(NeutralMode.Brake);
    motor.configOpenloopRamp(0.1);

    // 1/5 of a rev or better is consideted 'at desired position'
    pid.setTolerance(0.2);
    profiled_pid.setTolerance(0.2, 0.2);

    // When using I gain, consider setting this:
    // position_pid.setIntegratorRange(minimumIntegral, maximumIntegral);

    // Allow setting PID parameters on dashboard
    SmartDashboard.putData("PID", pid);
    SmartDashboard.setDefaultNumber("Profiled P", 0.15);
    SmartDashboard.setDefaultNumber("Profiled D", 0.0);

    SmartDashboard.setDefaultNumber("R per RPS", request_per_rps);

    SmartDashboard.setDefaultNumber("Turns", 10);
  }

  @Override
  public void robotPeriodic()
  {
    super.robotPeriodic();
    SmartDashboard.putNumber("Revs", getRevs());
    SmartDashboard.putNumber("RPS", getRPS());
  }

  @Override
  public void teleopPeriodic()
  {
    motor.set(0.25);
  }

  @Override
  public void autonomousInit()
  {
    super.autonomousInit();

    motor.setSelectedSensorPosition(0);
    pid.reset();
    profiled_pid.reset(getRevs());
  }
  
  @Override
  public void autonomousPeriodic()
  {
    // Toggle between two desired positions every 2 seconds
    final double desired_position = ((System.currentTimeMillis() / 2000) % 2) * SmartDashboard.getNumber("Turns", 1);

    double speed;
    
    // Plain PID
    speed = pid.calculate(getRevs(), desired_position);
    // SmartDashboard.putNumber("Position Error", pid.getPositionError());
    // SmartDashboard.putBoolean("At Position", pid.atSetpoint());

    // Profiled PID
    profiled_pid.setP(SmartDashboard.getNumber("Profiled P", 0.0));
    profiled_pid.setD(SmartDashboard.getNumber("Profiled D", 0.0));
    request_per_rps = SmartDashboard.getNumber("R per RPS", 0);
    speed = profiled_pid.calculate(getRevs(), desired_position);
    System.out.println(profiled_pid.getSetpoint().position);
    speed += profiled_pid.getSetpoint().velocity * request_per_rps;
    SmartDashboard.putNumber("Position Error", profiled_pid.getPositionError());
    SmartDashboard.putBoolean("At Position", profiled_pid.atGoal());

    motor.set(MathUtil.clamp(speed, -0.25, 0.25));
  }
}