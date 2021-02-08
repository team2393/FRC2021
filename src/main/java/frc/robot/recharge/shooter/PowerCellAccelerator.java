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
  private final DigitalInput shooter_sensor_mid = new DigitalInput(RobotMap.SHOOTER_SENSOR_READY);
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

  /** Possible States */
  public enum State
  {
    OFF(true),
    LOAD(true),
    SPIN_UP(true),
    EJECT(false),
    HOT(false),
    UN_JAM(true);

    /** May somebody request entering this state?
     *  Or do we automatically enter it?
     */
    public final boolean may_request;

    private State(boolean may_request)
    {
      this.may_request = may_request;
    }
  }

  /** Current state */
  private State state = State.OFF;

  /** Last state */
  private State last_state = null;


  /** Timer started when entering 'HOT' state to keep the ejector running */
  private final Timer keep_running_timer = new Timer();

  /** Timer for moving agitator up and down */
  private final Timer agitator_timer = new Timer();

  /** How long does the spinup take? */
  private final Timer spinup_timer = new Timer();


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
  
  public void setState(State desired)
  {
    if (desired.may_request)
      state = desired;
    else
      throw new IllegalStateException("You can't change state from " + state + " to " + desired);
  }
  
  public State getState()
  {
    return state;
  }

  /** Move top conveyor */
  private void moveTop(final double volt)
  {
    conveyor_top.setVoltage(volt);
  }
  
  /** Move bottom conveyor */
  private void moveBottom(final double volt)
  {
    conveyor_bottom.setVoltage(volt);
  }

  private void runAgitator(final boolean run)
  {
    if (run == false)
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
  }

  /** @return Is there a cell at the end of the horizontal conveyor? */
  public boolean isLowConveyorFull()
  {
    return !shooter_sensor_low_conveyor.get() || !shooter_sensor_mid.get();
  }

  public double getShooterRPM()
  {
    return shooter.getRPM();
  }

  /** Returns true if a power cell is being shot */
  public boolean powerCellFired()
  {
    return !shooter_sensor_eject.get();
  }

  @Override
  public void periodic()
  {
    // Entering new state?
    boolean enter = state != last_state;
    last_state = state;

    if (state == State.OFF)
    {
      shooter.setVoltage(0);
      moveTop(0);
      moveBottom(0);
      runAgitator(false);
    }

    if (state == State.LOAD)
    {
      shooter.setVoltage(0);
      moveTop(0);
      handleLoading();
    }

    if (state == State.SPIN_UP)
    {    
      if (enter)
      {
        spinup_timer.reset();
        spinup_timer.start();
        System.out.println("SPIN UP");
      }

      shooter.setRPM(SHOOTER_RPM);
      moveTop(0);
      handleLoading();

      // If fast enough, enter EJECT 
      if (getShooterRPM() >= MINIMUM_RPM_FRACTION * SHOOTER_RPM)
      {
        state = State.EJECT;
        enter = true;
        System.out.println("EJECT at " + getShooterRPM() + " RPMs after spinup of " + spinup_timer.get() + " seconds");
      }
    }

    if (state == State.EJECT)
    {
      shooter.setRPM(SHOOTER_RPM);
      moveTop(CONVEYOR_VOLTAGE);
      moveBottom(CONVEYOR_VOLTAGE);
      runAgitator(! isLowConveyorFull());

      // Has a ball been fired?
      if (powerCellFired())
      {
        state = State.HOT;
        enter = true;
      }
    }

    if (state == State.HOT)
    {
      if (enter)
      {
        // (Re-)start timer that keeps spinner running
        keep_running_timer.reset();
        keep_running_timer.start();
        System.out.println("Shot at " + getShooterRPM() + " RPMs");
      }

      shooter.setRPM(SHOOTER_RPM);
      moveTop(0);
      handleLoading();

      if (keep_running_timer.hasElapsed(2.0))
        state = State.LOAD;
    }
    
    if (state == State.UN_JAM)
    {
      shooter.setVoltage(0);
      // timer_on = false;
      moveTop(-CONVEYOR_VOLTAGE);
      moveBottom(-CONVEYOR_VOLTAGE);
      runAgitator(false);
    }

    SmartDashboard.putNumber("RPM", getShooterRPM());
  }

  /** Run bottom conveyor and agitator until we have one ball (or more) */
  private void handleLoading()
  {
    if (isLowConveyorFull())
    {
      moveBottom(0);
      runAgitator(false);
    }
    else
    {
      moveBottom(CONVEYOR_VOLTAGE);
      runAgitator(true);
    }
  }
}
