/*----------------------------------------------------------------------------*/
/* Copyright (c) 2020 FIRST Team 2393. All Rights Reserved.                   */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

/** Starting point for Team 2393 robots
 *  
 *  <p>A 'timed' robot that displays the class name on startup,
 *  so we can always see which code is running on the roboRIO.
 *  
 *  <p>It triggers the (new) command `scheduler` in robotPeriodic
 *  so that we can use the command framework.
 * 
 *  <p>It communicates with the FRC driver station,
 *  understands the different modes (disabled, autonomous, teleop),
 *  it just doesn't _do_ anything in the various modes beyond printing
 *  which mode it enters.
 *  
 *  <p>To implement a more functional robot,
 *  derive from this class and override the desired methods.
 *  
 *  <p>To select which code to actually run on the roboRIO,
 *  set the `robot.class` in build.properties
 */
public class BasicRobot extends TimedRobot
{
  /** Called once on startup */
  @Override
  public void robotInit()
  {
    // Print the class name (which might be the name of the derived class).
    // This way we can see what's running on the roboRIO console.
    System.out.println("###################################################");
    System.out.println("########### Team 2393");
    System.out.println("########### " + getClass().getName());
    System.out.println("###################################################");
  }

  /** Called periodically in ANY mode */
  @Override
  public void robotPeriodic()
  {
    CommandScheduler.getInstance().run();
  }

  /** Called when robot switches to DISABLED mode */
  @Override
  public void disabledInit()
  {
    System.out.println("Mode: Disabled");
  }

  /** Called periodically while DISABLED */
  @Override
  public void disabledPeriodic()
  {
    // Well, when we're disabled, we probably shouldn't do much...
  }

  /** Called when robot switches to AUTONOMOUS mode */
  @Override
  public void autonomousInit()
  {
    System.out.println("Mode: Auto-no-mouse");
  }

  /** Called periodically while in AUTONOMOUS mode */
  @Override
  public void autonomousPeriodic()
  {
  }

  /** Called when robot switches to TELEOP mode */
  @Override
  public void teleopInit()
  {
    System.out.println("Mode: Tele-op");
  }

  /** Called periodically while in TELEOP mode */
  @Override
  public void teleopPeriodic()
  {
  }

  /** Called when robot switches to TEST mode */
  @Override
  public void testInit()
  {
    System.out.println("Mode: Test");
  }

  /** Called periodically while in TEST mode */
  @Override
  public void testPeriodic()
  {
  }
}
