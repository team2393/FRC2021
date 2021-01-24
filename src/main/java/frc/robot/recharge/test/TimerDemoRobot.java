/*----------------------------------------------------------------------------*/
/* Copyright (c) 2020 FIRST Team 2393. All Rights Reserved.                   */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.recharge.test;

import edu.wpi.first.wpilibj.Timer;
import frc.robot.BasicRobot;

/** Robot code for testing ball handling
 *  Use _after_ SpinnerTestRobot has tuned spinner
 */
public class TimerDemoRobot extends BasicRobot
{
  @Override
  public void robotInit()
  {
    super.robotInit();

    final Timer timer = new Timer();
    // This will continue to accumulate time,
    // likely not what we want
    for (int i=0; i<10; ++i)
    {
      timer.start();
      Timer.delay(0.5);
      timer.stop();
      System.out.println("Start/stop: " + timer.get());
    }

    // This works for measuring time between start..end
    for (int i=0; i<10; ++i)
    {
      timer.reset();
      timer.start();
      Timer.delay(0.5);
      timer.stop();
      System.out.println("Reset/start/stop: " + timer.get());
    }

    // This works as well
    timer.start();
    for (int i=0; i<10; ++i)
    {
      timer.reset();
      Timer.delay(0.5);
      System.out.println("Start, then reset/get: " + timer.get());
    }
  }
}
