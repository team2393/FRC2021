/*----------------------------------------------------------------------------*/
/* Copyright (c) 2020 FIRST Team 2393. All Rights Reserved.                   */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.recharge.climb;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.recharge.RobotMap;

/** Motors related to climb mechanism */
public class Climber extends SubsystemBase
{
  private final WPI_TalonFX telescope = new WPI_TalonFX(RobotMap.TELESCOPE_MOTOR);
  private final WPI_TalonSRX climb = new WPI_TalonSRX(RobotMap.CLIMBER_MOTOR);
  public static final int max_height = 151704;

  public Climber()
  {
    telescope.configFactoryDefault();
    telescope.setSelectedSensorPosition(0);
    
    //Avoid ripping telescope cord
    telescope.configOpenloopRamp(0.5);
    climb.configFactoryDefault();
    climb.setInverted(true);
  }

  /** Move telescoping arm up/down
   *  @param direction 1: Full speed up, -1: Full speed down
   */
  public void moveTelescope(double direction)
  {
    telescope.set(-direction);
  }

  /** Pull up
   *  @param speed How fast to pull up, 0..1
   */
  public void pullUp(double speed)
  {
    if (speed >= 0)
      climb.set(speed);
    else
      climb.set(0);
    // Ratched prevents us from going back down.
  }

  public double getHeight()
  {
    return -telescope.getSelectedSensorPosition();
  }
}
