/*----------------------------------------------------------------------------*/
/* Copyright (c) 2020 FIRST Team 2393. All Rights Reserved.                   */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.recharge.shooter;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.BaseMotorController;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.recharge.RobotMap;

/** Power cell handling: Conveyor and ejector
 * 
 *  'Fuel cell' balls drop from hopper onto horizontal conveyor belt.
 *  Horizontal belt moves them to vertical conveyor,
 *  which feeds them to ejector/shooter.
 * 
 *  Maintains a spinner for shooting balls,
 *  keeping it running for a little longer after last shot
 *  in case we then soon need it again.
 */
public class PowerCellAccelerator extends SubsystemBase 
{
  // Motors
  private final Spinner shooter = new Spinner();
  private final WPI_VictorSPX conveyor_top = new WPI_VictorSPX(RobotMap.CONVEYOR_TOP);
  private final WPI_VictorSPX conveyor_bottom = new WPI_VictorSPX(RobotMap.CONVEYOR_BOTTOM); 
  
  // Sensors
  private final DigitalInput shooter_sensor_ready = new DigitalInput(RobotMap.SHOOTER_SENSOR_READY);
  private final DigitalInput shooter_sensor_eject = new DigitalInput(RobotMap.SHOOTER_SENSOR_EJECT);
  private final DigitalInput shooter_sensor_low_conveyor = new DigitalInput(RobotMap.SHOOTER_SENSOR_LOW_CONVEYOR);

  //Solenoids
  private final Solenoid agitator = new Solenoid(RobotMap.INTAKE_AGITATOR);

  /** Normal voltage for moving conveyors */
  public final static double CONVEYOR_VOLTAGE = 11.0;

  /** Ejector spinner setpoint */
  public static double SHOOTER_RPM = 5000;

  /** Minimum speed for shooting a ball as fraction of SHOOTER_RPM */
  public final static double MINIMUM_RPM_FRACTION = 0.99;

  private boolean unjammer = false;


  /** Should we auto-load?
   *  This will turn the conveyors on to get
   *  balls to the low and 'ready' sensors
   */
  private boolean auto_load = true;

  /** Should we feed the ejector spinner?
   *  This will keep the top conveyor running
   *  even if a ball is 'ready', i.e. move
   *  the 'ready' ball on into the ejector.
   */
  private boolean feed_shooter = false;

  /** Should we shoot?
   *  This runs the ejector as requested
   *  plus a little longer
   */
  private boolean shoot = false;
  /** Timer started when 'shoot' clears to keep the ejector running */
  private final Timer keep_running_timer = new Timer();

  /** Timer for moving agitator up and down */
  private final Timer agitator_timer = new Timer();

  /** Is the timer running? */
  private boolean timer_on = false;

  public PowerCellAccelerator()
  {
    commonSettings(conveyor_top, NeutralMode.Brake);
    commonSettings(conveyor_bottom, NeutralMode.Brake);
    agitator_timer.reset();
    agitator_timer.start();
  }
  
  /** @param motor Motor to configure with common settings
   *  @param mode Neutral mode
   */
  static void commonSettings(final BaseMotorController motor, final NeutralMode mode)
  {
    motor.configFactoryDefault();
    motor.clearStickyFaults();
    motor.setNeutralMode(mode);
    motor.setInverted(true);
  }
  
  /** @param enable Enable loading? */
  public void enableLoad(final boolean enable)
  {
    auto_load = enable;
  }

  public void unjam(boolean unjamming_enabled)
  {
    unjammer = unjamming_enabled;
  }

  /** Move top conveyor */
  public void moveTop(final double volt)
  {
    conveyor_top.setVoltage(volt);
  }
  
  /** Move bottom conveyor */
  public void moveBottom(final double volt)
  {
    if (volt == 0)
    {
      agitator.set(false);
      agitator_timer.reset();
      agitator_timer.start();
    }
    else
    {
      boolean agitator_up = agitator.get();
      if (!agitator_up && agitator_timer.hasElapsed(1.0))
      {
        // Was down for 1 second: Move up, reset timer
        agitator.set(true);
        agitator_timer.reset();
        agitator_timer.start();
      }
      else if (agitator_up && agitator_timer.hasElapsed(0.2))
      {
        // Was up for 0.2 secs: Move down, reset timer
        agitator.set(false);
        agitator_timer.reset();
        agitator_timer.start();
      }
    }
    conveyor_bottom.setVoltage(volt);
  }
  
  /** @return Is cell in "ready" position at end of vertical conveyor? */
  public boolean isPowerCellReady()
  {
    return !shooter_sensor_ready.get();
  }

  /** @return Is there a cell at the end of the horizontal conveyor? */
  public boolean isLowConveyorFull()
  {
    return !shooter_sensor_low_conveyor.get();
  }

  /** Turn shooter 'on' or 'off'.
   * 
   *  When turned 'off', it actually remains
   *  running for a few seconds so in case
   *  we want to turn it 'on' again it's
   *  already up to speed.
   */
  public void eject(final boolean on_off)
  {
    // If shooter was on and is now requested off,
    // start timer so it keeps running for a little longer
    if (shoot == true  &&  on_off == false)
    {
      keep_running_timer.reset();
      keep_running_timer.start();
      timer_on = true;
    }
        
    shoot = on_off;
  }

  public double getShooterRPM()
  {
    return shooter.getRPM();
  }

  public void feedEjector(final boolean do_it)
  {
    feed_shooter = do_it;
  }

  /** Returns true if a power cell is being shot */
  public boolean powerCellFired()
  {
    return !shooter_sensor_eject.get();
  }

  @Override
  public void periodic()
  {
    if (unjammer)
    {
      shooter.setVoltage(0);
      shoot = false;
      timer_on = false;
      moveBottom(-CONVEYOR_VOLTAGE);
      moveTop(-CONVEYOR_VOLTAGE);
      return;
    }

    // Run ejector if we're asked to do it,
    // or for 2 more seconds after the last shot
    // so it remains running through a series of shots
    if (shoot  ||  (timer_on && !keep_running_timer.hasElapsed(2.0)))
      shooter.setRPM(SHOOTER_RPM);
    else
    {
      shooter.setVoltage(0);
      timer_on = false;
    }
    SmartDashboard.putNumber("RPM", getShooterRPM());

    if (auto_load)
    {
      // Keep horiz. belt moving until ball is in there,
      // regardless of what the vertical conveyor and ejector are doing.
      if (! isLowConveyorFull()  ||  ! isPowerCellReady())
        moveBottom(CONVEYOR_VOLTAGE);
      else
        moveBottom(0);

      // Move vertical belt to feed a ball to the shooter,
      // or until a ball is ready for the next shot
      if (feed_shooter   ||   ! isPowerCellReady())
        moveTop(CONVEYOR_VOLTAGE);
      else
        moveTop(0);
     }
     else
     {
       moveTop(0);
       moveBottom(0);
     }
  }
}
