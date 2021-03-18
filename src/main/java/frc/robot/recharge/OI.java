/*----------------------------------------------------------------------------*/
/* Copyright (c) 2020 FIRST Team 2393. All Rights Reserved.                   */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.recharge;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.Hand;

/**
 * Operator Interface Definitions
 * 
 * One place to find which button does what
 */
public class OI
{
  public static final XboxController joystick = new XboxController(0);
  public static final Joystick buttonboard = new Joystick(1);

  /** Reset joystick memory */
  public static void reset()
  {
    // The 'getXXXPressed' methods remember if a button was
    // pressed, even just briefly.
    // Trouble is this includes times when it was pressed while
    // we were disabled, and then suddenly things start to happen
    // as we enable..
    // --> Read each 'pressed' state once to clear it
    for (int i=1; i<=10; ++i)
      joystick.getRawButtonPressed(i);

    for (int i=1; i<=20; ++i)
      buttonboard.getRawButtonPressed(i);
  }

  public static boolean selectNear()
  {
    return buttonboard.getRawButtonPressed(12);
  }

  public static boolean selectMid()
  {
    return buttonboard.getRawButtonPressed(13);
  }

  public static boolean selectFar()
  {
    return buttonboard.getRawButtonPressed(14);
  }

  public static boolean selectDriveMode()
  {
    return buttonboard.getRawButton(3);
  }

  public static boolean selectClimbMode()
  {
    return buttonboard.getRawButton(5);
  }
  
  public static boolean selectWheelMode()
  {
    return buttonboard.getRawButton(4);
  }

  public static boolean getUnjam()
  {
    return buttonboard.getRawButton(11);
  }
  
  public static boolean isLowGearRequested()
  {
    return joystick.getTriggerAxis(Hand.kRight) > .5;
  }

  public static boolean isHighGearRequested()
  {
    return joystick.getBumperPressed(Hand.kRight);
  }

  public static boolean force_low_speed = false;

  private static double getSpeedFactor()
  {
    if (force_low_speed  ||  joystick.getTriggerAxis(Hand.kRight) > 0.6)
      return 0.5;
    else
      return 1;
  }
  
  /** 'Signed square' to get more sensitivity around joystick center
   *  
   *  Same idea as in DifferentialDrive.arcadeDrive,
   *  but drive train passes the speed & rotation values
   *  on un-squared to keep PID-based moves linear.
   *  Instead, we square the joystick values so that interactive
   *  drive is de-sensitized.
   */
  private static double square(final double value)
  {
    return Math.signum(value) * (value * value);
  }

  /** @return Speed (1=full ahead) */
  public static double getSpeed()
  {
    double speed = square(getSpeedFactor() * -joystick.getY(Hand.kLeft));
    // speed = speed_limiter.calculate(speed);
    return speed;
  }

  /** @return Left/right steering */
  public static double getDirection()
  {
    if (joystick.getStickButton(Hand.kRight))
      return 0;
    else
      return square(getSpeedFactor() * joystick.getX(Hand.kRight));
  }
  
  public static final boolean isAlignOnTargetHeld()
  {
    return joystick.getYButton();
  }
  public static final boolean isIntakeTogglePressed()
  {
    return joystick.getXButton();
  }
  public static final boolean isShootHeld()
  {
    return joystick.getAButton();
  }

  /** @return Up/Down for hood */
  public static double getHoodSpeed()
  {
    if (!joystick.getStickButton(Hand.kRight))
      return 0;
    else
      return square(joystick.getX(Hand.kRight));
  }

  public static boolean isAutorotateWheelRequested()
  {
    return buttonboard.getRawButtonPressed(9);
  }
  
  public static boolean toggleHood()
  {
    // TODO return buttonboard.getRawButtonPressed(button)
    return joystick.getRawButtonPressed(7);
  }   

  public static boolean isRotateToColorRequested()
  {
    return buttonboard.getRawButtonPressed(10);
  }

  public static boolean prepareShooter()
  {
    // TODO replace button with on/off switch
    return buttonboard.getRawButton(10);
  }

  /** @return Manual fortune wheel speed */
  public static double getWheelSpeed()
  {
    return buttonboard.getRawButton(8) ? 0.2 : 0.0;
  }
  
  /** @return telescope speed -1 (down) to 1 (up) */
  public static double getTelescopeSpeed()
  {
    if (buttonboard.getRawButton(1))
      return -0.7;
    else if (buttonboard.getRawButton(2))
      return 0.9;
    else 
      return 0;
  }

  public static double getClimbSpeed()
  {
    return joystick.getTriggerAxis(Hand.kRight);    
  }
}