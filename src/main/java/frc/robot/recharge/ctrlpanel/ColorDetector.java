/*----------------------------------------------------------------------------*/
/* Copyright (c) 2020 FIRST Team 2393. All Rights Reserved.                   */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/
package frc.robot.recharge.ctrlpanel;

/** Basic color detector interface */
public interface ColorDetector
{
  /** Color names for color index 0, 1, 2, 3 */
  public enum Segment_Color
  {
    Unkown,
    Blue,
    Green,
    Red,
    Yellow;

    public Segment_Color next()
    {
      if (this == Blue)
        return Segment_Color.Green;
      else if (this == Green)
        return Segment_Color.Red;
      else if (this == Red)
        return Segment_Color.Yellow;
      else if (this == Yellow)
        return Segment_Color.Blue;
      else
        return Segment_Color.Unkown;
    }
  }

  /** @return Color */
  Segment_Color getColor();
}
