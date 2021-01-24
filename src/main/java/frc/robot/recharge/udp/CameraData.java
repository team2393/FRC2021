/*----------------------------------------------------------------------------*/
/* Copyright (c) 2020 FIRST Team 2393. All Rights Reserved.                   */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.recharge.udp;

/** Information received from camera */
public class CameraData 
{
  /** Time in milliseconds when data was received */
  public long millisec;

  /** Horizontal direction to target, 'right' is positive */
  public int direction;

  /** Vertical indicator of distance to target, 'up' is positive */
  public int distance;

  public CameraData(final int direction, final int distance)
  {
    this(System.currentTimeMillis(), direction, distance);
  }

  public CameraData(final long millisec, final int direction, final int distance)
  {
    this.millisec = millisec;
    this.direction = direction;
    this.distance = distance;
  }

  @Override
  public String toString() 
  {
    return (millisec + ": Direction: " + direction + " Distance: " + distance);
  }
}
