/*----------------------------------------------------------------------------*/
/* Copyright (c) 2020 FIRST Team 2393. All Rights Reserved.                   */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.recharge.shooter;

import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.recharge.RobotMap;

/** Power cell handling: Ejector hood
 * 
 *  Angle of rotatable hood/shield/deflector adjusts
 *  the angle at which balls are ejected.
 * 
 *  Angle is calibrated by resetting encoder at startup,
 *  i.e. hood must be all the way 'down' when powered on.
 */
public class Hood extends SubsystemBase
{
  // Motor, must have encoder (angle)
  private final Solenoid hood = new Solenoid(RobotMap.HOOD_ADJUST);

  public Hood()
  {
    hood.set(false);
  }
  
  /** @return Hood position */
  public boolean getHoodPosition()
  {
    return hood.get();
  }

  /** @param position Set hood up or down */
  public void set(final boolean hood_up)
  {
    hood.set(hood_up);
  }

  @Override
  public void periodic()
  {
    SmartDashboard.putBoolean("Hood Up", getHoodPosition());
  }
}
