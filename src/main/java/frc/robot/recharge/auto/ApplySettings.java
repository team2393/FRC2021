/*----------------------------------------------------------------------------*/
/* Copyright (c) 2020 FIRST Team 2393. All Rights Reserved.                   */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.recharge.auto;

import java.io.BufferedReader;
import java.io.File;
import java.io.FileReader;
import java.util.HashMap;
import java.util.Map;

import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;

/** Apply settings from file to network tables */
public class ApplySettings extends InstantCommand
{
  // Value could be Double, Boolean or String
  private final Map<String, Object> settings = new HashMap<>();

  public ApplySettings(final String filename)
  {
    final File file = new File(Filesystem.getDeployDirectory(), filename);
    System.out.println("Settings file: " + file);
    try
    (
      final BufferedReader reader = new BufferedReader(new FileReader(file))
    )
    {
      // Read the settings from file
      for (String line = reader.readLine();
           line != null;
           line = reader.readLine())
      {
        // Remove leading and trailing spaces
        line = line.trim();
        // Ignore comments
        if (!line.isBlank()  &&  !line.startsWith("#"))
        {
          // Chop "Some Name      value"  into "Some Name" and "value"
          final int sep = line.lastIndexOf(" ");
          if (sep < 0)
             continue;
          final String setting = line.substring(0, sep).trim();
          final String value = line.substring(sep+1).trim();
          System.out.println("Reading " + setting + " = " + value);

          // Remember for later when we execute()
          try
          {
            double number = Double.parseDouble(value);
            settings.put(setting, number);
          }
          catch (NumberFormatException ex)
          {
            // Not a number...
            // Check if it's a boolean
            if (value.equalsIgnoreCase("true"))
              settings.put(setting, Boolean.TRUE);
            else if (value.equalsIgnoreCase("false"))
              settings.put(setting, Boolean.FALSE);
            else // fall back to String
              settings.put(setting, value);
          }
        }
      }
    }
    catch (Exception ex)
    {
      System.out.println("Cannot read " + file);
      ex.printStackTrace();
    }
  }

  @Override
  public boolean runsWhenDisabled()
  {
    return true;
  }

  @Override
  public void execute()
  {
    // Apply the settings!
    System.out.println("Applying settings:");
    for (String setting : settings.keySet())
    {
      Object value = settings.get(setting);
      if (value instanceof Double)
        SmartDashboard.putNumber(setting, (Double) value);
      else if (value instanceof Boolean)
        SmartDashboard.putBoolean(setting, (Boolean) value);
      else
        SmartDashboard.putString(setting, (String) value);
      System.out.println("Setting " + setting + " to " + value);
    }
  }
}
