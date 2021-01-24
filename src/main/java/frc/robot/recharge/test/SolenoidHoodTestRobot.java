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
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.BasicRobot;
import frc.robot.recharge.OI;
import frc.robot.recharge.shooter.Hood;

/** Robot code for testing hood solenoid */
public class SolenoidHoodTestRobot extends BasicRobot
{
  private final Hood hood = new Hood();

  private final CommandBase hood_up = new InstantCommand(() -> hood.set(true));
  private final CommandBase hood_down = new InstantCommand(() -> hood.set(false));
  private final Timer timer =  new Timer();

  @Override
  public void robotInit()
  {
    super.robotInit();
    SmartDashboard.putData("Hood Up", hood_up);
    SmartDashboard.putData("Hood Down", hood_down);
  }

  @Override
  public void robotPeriodic() 
  {
     super.robotPeriodic();
     SmartDashboard.putBoolean("Hood State", (hood.getHoodPosition()));
  }

  @Override
  public void disabledInit()
  {
    super.disabledInit();
    hood.set(false);
  }

  @Override
  public void teleopPeriodic()
  {
    // Hood solenoid is on as long as X button is held
    if (OI.toggleHood())
      if (!hood.getHoodPosition())
        hood_up.schedule();
      else
        hood_down.schedule();
  }

  @Override
  public void autonomousInit() 
  {
    super.autonomousInit();
    timer.reset();
    timer.start();
  }

  @Override
  public void autonomousPeriodic()
  {
    //TODO not sure if I used the timer correctly 
    if(timer.get() >= 0.5)
    {
      timer.reset();
      timer.start();
    
      if (hood.getHoodPosition())
        hood_down.schedule();
      else
        hood_up.schedule();
    }

    SmartDashboard.putBoolean("Hood Up", (hood.getHoodPosition()));

  }
}