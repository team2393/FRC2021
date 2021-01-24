/*----------------------------------------------------------------------------*/
/* Copyright (c) 2020 FIRST Team 2393. All Rights Reserved.                   */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.recharge.test;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.BasicRobot;
import frc.robot.recharge.OI;
import frc.robot.recharge.ctrlpanel.ControlWheel;
import frc.robot.recharge.ctrlpanel.ExtendControlWheel;
import frc.robot.recharge.ctrlpanel.ManualWheelSpeed;
import frc.robot.recharge.ctrlpanel.RetractControlWheel;
import frc.robot.recharge.ctrlpanel.RotateToColor;
import frc.robot.recharge.ctrlpanel.RotateWheel;

/** Robot code for testing control wheel */
public class ControlWheelTestRobot extends BasicRobot
{
  private final ControlWheel wheel = new ControlWheel();
  private final CommandBase manual = new ManualWheelSpeed(wheel);
  private final CommandBase auto = new RotateWheel(wheel, 3);
  private final CommandBase color = new RotateToColor(wheel);
    
  @Override
  public void robotPeriodic()
  {
    super.robotPeriodic();
    SmartDashboard.putString("Color", wheel.getColor().name());
  }
  
  @Override
  public void teleopPeriodic()
  {
    if (OI.selectWheelMode())
      new ExtendControlWheel(wheel).schedule();
    else if (OI.selectDriveMode() || OI.selectClimbMode())
      new RetractControlWheel(wheel).schedule();
    
    if (OI.isAutorotateWheelRequested())
      auto.schedule();
    if (OI.isRotateToColorRequested())
      color.schedule();
    if (! (auto.isScheduled()  ||  color.isScheduled()))
      manual.schedule();
  }
}
