/*----------------------------------------------------------------------------*/
/* Copyright (c) 2020 FIRST Team 2393. All Rights Reserved.                   */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.recharge.ctrlpanel;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.recharge.ctrlpanel.ColorDetector.Segment_Color;

/** Command to rotate wheel to desired color */
public class RotateToColor extends CommandBase
{
  private final ControlWheel wheel;

  private static final int REDUNDANCY = 3;
  private int times = 0;
  /** The color that we should find,
   *  or -1 if we don't know where to go.
   */
  private Segment_Color desired_color;

  private boolean is_finished;

  /** @param wheel Control wheel to turn */
  public RotateToColor(final ControlWheel wheel)
  {
    this.wheel = wheel;
    addRequirements(wheel);
  }

  @Override
  public void initialize()
  {
    // Determine which color we should go to
    times = 0;
    desired_color = getDesiredColor();
    if (desired_color == Segment_Color.Unkown)
    {
      is_finished = true;
      System.err.println("RotateToColor: Desired color is " + desired_color);
    }
    else
    {
      System.out.println("RotateToColor: Desired color is " + desired_color);
      wheel.fast();
      is_finished = false;
    }
  }
  
  private Segment_Color getDesiredColor()
  {
    /*
      Game Data   Our Position  Our Value
      Red         Blue          0
      Yellow      Green         1
      Blue        Red           2
      Green       Yellow        3
    */
    final String gameData = DriverStation.getInstance().getGameSpecificMessage();
    if (gameData.length() < 1)
      return Segment_Color.Unkown;
    if (gameData.charAt(0) == 'B')
      return Segment_Color.Red;
    if (gameData.charAt(0) == 'G')
      return Segment_Color.Yellow;
    if (gameData.charAt(0) == 'R')
      return Segment_Color.Blue;
    if (gameData.charAt(0) == 'Y')
      return Segment_Color.Green;
    return Segment_Color.Unkown;
  }

  @Override
  public void execute()
  {
    if (desired_color == Segment_Color.Unkown)
      return;

    // Did the camera detect a color?
    Segment_Color color = wheel.getColor();
    if (color == Segment_Color.Unkown)
    {
      // System.out.println("Unknown color");
      // Go slow to improve chance of catching that color
      wheel.slow();
      return;
    }

    // Have we reached the expected color?
    if (color == desired_color)
    {
      if (++times >= REDUNDANCY)
      {
        is_finished = true;
        System.out.println("Found " + color.toString());
      }
      else
        System.out.println("Verified " + desired_color + " " + times + "  times");
    }
    else
      wheel.fast();
  }

  @Override
  public void end(final boolean interrupted)
  {
    wheel.spin(0);
  }

  @Override
  public boolean isFinished()
  {
    return is_finished;
  }
}
