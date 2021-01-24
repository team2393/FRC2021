/*----------------------------------------------------------------------------*/
/* Copyright (c) 2020 FIRST Team 2393. All Rights Reserved.                   */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.recharge.udp;

import frc.robot.recharge.util.CircularBuffer;

/** Guess 'current' camera data
 * 
 *  Remembers the last two updates that we received
 *  from the camera.
 *  Assumes that we can draw a line from the last two
 *  data points over time to 'now' to have a reasonable
 *  guess for the current value.
 */
public class Guesstimator 
{
  /** If the oldest camera update is half a second old, forgettaboutit */
  private static final long TOO_OLD_MS = 500;

  /** Ring buffer for the last two camera updates */
  private CircularBuffer<CameraData> updates = new CircularBuffer<>(2);

  /** @param update Latest camera update */
  public void addData(final CameraData update)
  {
    updates.add(update);
  }

  /** Extrapolate from two data points onto another time 
   * 
   *  @param t1 Time when first data point was taken
   *  @param v1 Value of first point
   *  @param t2 Time when second data point was taken
   *  @param v2 Value of second point
   *  @param t Time for which we want to estimate the value
   *  @return Estimated value
   */
  private double extrapolate(double t1, double v1, double t2, double v2, double t)
  {
    // Compute line through points (t1, v1)  and  (t2, v2),
    // i.e. x axis is 'time', and y axis for value

    // 1) Compute slope
    final double slope = (v2 - v1) / (t2 - t1);

    // 2) Compute value axis intersection, either via
    //    v1 = v0 + slope * t1  or
    //    v2 = v0 + slope * t2
    final double v0 = v1 - slope * t1;
    
    // Now get value at requested time
    return v0 + slope * t;
  }

  /** Try to predict what the camera should see right now,
   *  based on the last two updates that we received.
   * 
   *  @return Estimated {@link CameraData}, set to zero direction & distance if we have no clue
   */
  public CameraData guesstimate()
  {
    final long now = System.currentTimeMillis();

    // Do we have two updates?
    if (updates.size() < 2)
      return new CameraData(now, 0, 0);
    
    final CameraData oldest = updates.get(0);
    final CameraData last = updates.get(1);

    // Are they fresh enough?
    if (oldest.millisec < now - TOO_OLD_MS)
      return new CameraData(now, 0, 0);

    // Draw line through the old points, then get value 'now'
    if (oldest.millisec == last.millisec)
    {
      // This would result in division-by-zero when computing the slope..
      System.out.println("Guesstimator: Time is not progressing");
      return new CameraData(now, 0, 0);
    }
    final double direction = extrapolate(oldest.millisec, oldest.direction,
                                         last.millisec, last.direction,
                                         now);
    final double distance = extrapolate(oldest.millisec, oldest.distance,
                                        last.millisec, last.distance,
                                        now);
    return new CameraData(now, (int) direction, (int) distance);
  }

  public static void main(String[] args)
  {
    Guesstimator guesstimator = new Guesstimator();

    // Simulate camera updates every 1000/30 = 33 ms
    final long now = System.currentTimeMillis();
    guesstimator.addData(new CameraData(now-66, 100,  0));  
    guesstimator.addData(new CameraData(now-33,  90, 10));
    
    // Guess what the next update would be if the same trend continued
    CameraData guess = guesstimator.guesstimate();
    System.out.println(guess);
    System.out.println("^^ Should be close to Direction 80, distance 20 ^^");
  }
}
