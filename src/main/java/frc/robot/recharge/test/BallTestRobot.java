/*----------------------------------------------------------------------------*/
/* Copyright (c) 2020 FIRST Team 2393. All Rights Reserved.                   */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.recharge.test;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.BasicRobot;
import frc.robot.recharge.OI;
import frc.robot.recharge.shooter.Eject;
import frc.robot.recharge.shooter.PowerCellAccelerator;

/** Robot code for testing ball handling
 *  Use _after_ SpinnerTestRobot has tuned spinner
 */
public class BallTestRobot extends BasicRobot
{
  private final PowerCellAccelerator pca = new PowerCellAccelerator();

  @Override
  public void robotInit()
  {
    super.robotInit();
    // Lower RPM for tests to be gentle on ears and equipment
    PowerCellAccelerator.SHOOTER_RPM = 2000; 
  }

  @Override
  public void robotPeriodic()
  {
    super.robotPeriodic();
    // 2) Check spinner RPM when A is held

    // 5) Attach prox sensor for "Ready" such that it detects
    //    when ball has been moved up to just-before the ejector
    // SmartDashboard.putBoolean("Ready", pca.isPowerCellReady());
    // 6) Attach prox sensor for "Fired" such that it detects
    //    a ball in the ejector.
    //    Check this first without running the ejector,
    //    when with both A and Y held to see if a ball
    //    is detected during ejection
    SmartDashboard.putBoolean("Fired", pca.powerCellFired());

    SmartDashboard.putBoolean("Low Conveyor Full", pca.isLowConveyorFull());
  }

  @Override
  public void teleopInit()
  {
    super.teleopInit();
    pca.enableLoad(false);
  }

  @Override
  public void teleopPeriodic()
  {
    // 1) Test if ejector runs when A is held (plus a little longer)
    //     (Spinner speed and direction already adjusted via SpinnerTestRobot)
    pca.eject(OI.joystick.getAButton());

    // 3) Check if 'forward' moves conveyors in correct direction
    //    at suitable speed.
    // 4) Hold Y to run run conveyor at its 'normal' speed
    if (OI.joystick.getYButton())
    { 
      pca.moveBottom(PowerCellAccelerator.CONVEYOR_VOLTAGE);
      pca.moveTop(PowerCellAccelerator.CONVEYOR_VOLTAGE);
    }
    else
    {
      pca.moveTop(12*OI.getSpeed());
      pca.moveBottom(12*OI.getDirection());
      System.out.println("Voltage Top " + 12*OI.getSpeed() + "Voltage Bottom" + 12*OI.getDirection());
    }
  }

  @Override
  public void autonomousInit() 
  {
    pca.enableLoad(true);
    SequentialCommandGroup sequence = new SequentialCommandGroup();
    sequence.addCommands(new Eject(pca));
    sequence.addCommands(new Eject(pca));
    sequence.addCommands(new Eject(pca));
    sequence.schedule();
  }
}
