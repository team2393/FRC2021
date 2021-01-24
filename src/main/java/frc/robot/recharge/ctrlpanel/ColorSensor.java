/*----------------------------------------------------------------------------*/
/* Copyright (c) 2020 FIRST Team 2393. All Rights Reserved.                   */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/
package frc.robot.recharge.ctrlpanel;

import com.revrobotics.ColorSensorV3;

import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;

/** REV Robotics color sensor
 * 
 *  Info is on http://www.revrobotics.com/rev-31-1557
 * 
 *  To get the supporting library,
 *  downloaded the REVColorSensorV3.json into
 *  C:\Users\Public\wpilib\2020\vendordeps,
 *  then invoked View, Command Palette,  WPIlib Manage Vendor Libraries,
 *  Add new library (offline) and select REVColorSensorV3
 */
public class ColorSensor implements ColorDetector
{
  /** Sensor, connected to I2C socket on RIO */
  private final ColorSensorV3 sensor = new ColorSensorV3(I2C.Port.kOnboard);

  /** RGB values of the expected colors */
  private final int[][] colors_rgb =
  { // R   G    B
    {  50, 120,  80 }, // Blue/Cyan
    {  50, 150,  60 }, // Green
    { 120,  90,  40 }, // Red
    {  80, 130,  30 }  // Yellow
  };

  /** Read color sensor
   *  @return Detector color index or -1
   */
  @Override
  public Segment_Color getColor()
  {
    final Color color = sensor.getColor();
    // Scale sensor's value range to 0..255
    final int red   = (int) (color.red * 255);
    final int green = (int) (color.green * 255);
    final int blue  = (int) (color.blue * 255);

    // Compare with expected colors.
    // Initial value for closest_dist determines
    // how well a color must match.
    int closest_dist = Integer.MAX_VALUE;
    int best_match = -1;
    for (int i=0; i<colors_rgb.length; ++i)
    {
      // Distance of RGB from expected color
      final int dr = red   - colors_rgb[i][0];
      final int dg = green - colors_rgb[i][1];
      final int db = blue  - colors_rgb[i][2];
      // Check for closest squared distance
      final int squared_dist = dr*dr + dg*dg + db*db;
      if (squared_dist < closest_dist)
      {
        closest_dist = squared_dist;
        best_match = i;
      }
    }
    final Segment_Color segment = Segment_Color.values()[best_match+1];

    final int dist = sensor.getProximity();
    if (closest_dist > 400)
    {
      SmartDashboard.putString("ColorSensor",
                               "Ignore " + segment + " (match " + closest_dist + ") RGB: " + red + ", " + green + ", " + blue + " at " + dist);
      return Segment_Color.Unkown;
    }

    SmartDashboard.putString("ColorSensor",
                             segment + " (match " + closest_dist + ") RGB: " + red + ", " + green + ", " + blue + " at " + dist);
      
    return segment;
  }
}