/*----------------------------------------------------------------------------*/
/* Copyright (c) 2020 FIRST Team 2393. All Rights Reserved.                   */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.recharge.test;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.BasicRobot;
import frc.robot.recharge.OI;
import frc.robot.recharge.shooter.Spinner;

/** Robot code for testing spinner */
public class SpinnerTestRobot extends BasicRobot
{
  private final Spinner spinner = new Spinner();
  private final Timer timer = new Timer();
  private double voltage;

  @Override
  public void robotInit()
  {
    super.robotInit();
    SmartDashboard.setDefaultNumber("kV", 0.00188);
    SmartDashboard.setDefaultNumber("P", 0.0);
  }

  @Override
  public void robotPeriodic()
  {
    super.robotPeriodic();
    SmartDashboard.putNumber("Angle", spinner.getAngle());
    SmartDashboard.putNumber("RPM", spinner.getRPM());
  }

  @Override
  public void teleopPeriodic()
  {
    // +- 12 Volts based on joystick
    voltage = (OI.getSpeed() * 12);
    System.out.println("RPM: " + spinner.getRPM() + " Voltage: " + (voltage));
    spinner.setVoltage(voltage);
  }

  @Override
  public void autonomousInit()
  {
    super.autonomousInit();
    voltage = 0.0;
    timer.reset();
    timer.start();
  }

  @Override
  public void autonomousPeriodic()
  {
    // Printouts for 'table' of RPM vs. voltage
    // spinner.setVoltage(voltage);
    // if (timer.hasPeriodPassed(2.0))
    // {
    //   System.out.println("RPM: " + spinner.getRPM() + " Voltage: " + voltage);
    //   voltage += 2;
    //   if (voltage > 12.1)
    //      voltage = 0.0;
    // }

    // Tune PID, then pick some reasonable RPM values between which to toggle
    spinner.configure(SmartDashboard.getNumber("kV", 0),
                      SmartDashboard.getNumber("P", 0));

    final boolean high = (System.currentTimeMillis() / 3000) % 2 == 0;
    spinner.setRPM(high ? 3000 : 5000);
  }
}
