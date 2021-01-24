/*----------------------------------------------------------------------------*/
/* Copyright (c) 2020 FIRST Team 2393. All Rights Reserved.                   */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.recharge.sound;

import java.util.List;

import com.ctre.phoenix.motorcontrol.can.TalonFX;

/** Command that plays a really cool beep */
public class BeepBipBipBeeeep extends PlayNotes
{  
  public BeepBipBipBeeeep(final List<TalonFX> instruments)
  {
    super(instruments,
          3,
          new int[]
          { 
            5*C4,
            OFF,
            OFF,
            5*C4,
            OFF,
            5*C4,
            OFF,
            5*C5,
            5*C5,
            5*C5,
            OFF,
            OFF,
            OFF,
            OFF
          });
  }
}
