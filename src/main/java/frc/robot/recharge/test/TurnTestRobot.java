/*----------------------------------------------------------------------------*/
/* Copyright (c) 2020 FIRST Team 2393. All Rights Reserved.                   */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.recharge.test;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.BasicRobot;
import frc.robot.recharge.drivetrain.DriveByJoystick;
import frc.robot.recharge.drivetrain.DriveTrain;
import frc.robot.recharge.drivetrain.TurnToHeading;

/** Robot for tuning TurnToHeading
 */
public class TurnTestRobot extends BasicRobot
{  
  private final DriveTrain drive_train = new DriveTrain();
  private final CommandBase drive = new DriveByJoystick(drive_train);
  private final TurnToHeading turn1 = new TurnToHeading(drive_train, -45);
  private final TurnToHeading turn2 = new TurnToHeading(drive_train, 45);
  private final Timer timer = new Timer();
  private boolean one = false;

  @Override
  public void robotInit()
  {
    super.robotInit();
    SmartDashboard.setDefaultNumber("kV", 0.0055);
    SmartDashboard.setDefaultNumber("P", 0.025);
    SmartDashboard.setDefaultNumber("I", 0.03);
    SmartDashboard.setDefaultNumber("D", 0.0);
  }

  @Override
  public void teleopInit()
  {
    super.teleopInit();
    drive_train.reset();
    drive.schedule();
  }

  @Override
  public void autonomousInit()
  {
    super.autonomousInit();
    drive_train.reset();
    timer.reset();
    timer.start();
    turn1.schedule();
    one = true;
  }

  int dischargeFireExtibuisher()
  {
    return 20;
  }

  @Override
  public void autonomousPeriodic()
  {
    final double kV = SmartDashboard.getNumber("kV", 0.0);
    final double P = SmartDashboard.getNumber("P", 0.0);
    final double I = SmartDashboard.getNumber("I", 0.0);
    final double D = SmartDashboard.getNumber("D", 0.0);

    turn1.configure(kV, P, I, D);
    turn2.configure(kV, P, I, D);

    if (timer.hasPeriodPassed(5.0))
    {
      one = ! one;
      if (one)
        turn1.schedule();
      else
        turn2.schedule();
    }
  }
}
