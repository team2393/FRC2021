/*----------------------------------------------------------------------------*/
/* Copyright (c) 2020 FIRST Team 2393. All Rights Reserved.                   */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.recharge.test;

import java.util.List;

import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.music.Orchestra;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.BasicRobot;
import frc.robot.recharge.sound.BeepBipBipBeeeep;
import frc.robot.recharge.util.PingSubnet;

/** Robot code for testing devices */
public class TestRobot extends BasicRobot
{
  // private final DigitalInput ball = new DigitalInput(8);
  // private final LEDStrip led = new LEDStrip();

  private final List<TalonFX> instruments = List.of(new TalonFX(1), new TalonFX(2), new TalonFX(3), new TalonFX(4));
  private final Orchestra orch = new Orchestra(instruments);
  private final CommandBase beep = new BeepBipBipBeeeep(instruments);

  @Override
  public void robotInit()
  {
    new PingSubnet();
  }

  @Override
  public void teleopInit()
  {
    super.teleopInit();
    beep.schedule();
  }

  @Override
  public void autonomousInit()
  {
    super.autonomousInit();
    System.out.println(orch.loadMusic("tune.chrp"));
    System.out.println(orch.play());
  }
}
