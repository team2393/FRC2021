/*----------------------------------------------------------------------------*/
/* Copyright (c) 2020 FIRST Team 2393. All Rights Reserved.                   */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/
package frc.robot.recharge;

/** Hardware Mappings
 *  
 *  One place to find what's connected how
 */ 
public class RobotMap
{    
  // Power Distribution Panel
  //
  // 40 Amp connectors
  // 0 Drive motor                 15 Drive motor
  // 1 Drive motor                 14 Drive motor
  // 2 Hood                        13 Shooter
  // 3 Telescope                   12 Colorspin  
  //
  // Below 40 Amp ports
  // 4 Intake Arm                  11 Intake Arm
  // 5 Intake Roller               10 Conveyor Back
  // 6 Climb                        9 Conveyor Horizontal
  // 7 Color LED                    8 Camera LED
  //
  // PDP controller port -> RoboRIO
  // PDP PCM port -> PCM, compressor, solenoids
  //
  // PDP VRM port ->
  // VRM 12V, 2A
  // 1) Radiop
  // 2) must not be used
  //
  // VRM 12V, 500mA
  // 1) Prox sensor at end of conveyors
  // 2) Prox sensor in 'ejector'
  //
  // VRM 5V, 500mA
  // 1)
  // 2)
  //
  // VRM 5V, 2A
  // 1) Raspberry Pi power
  // 2) Color LED strip power

  // Talon CAN IDs =================================
  // Drivetrain motors
  public final static int LEFT_MOTOR_MAIN = 4;
  public final static int RIGHT_MOTOR_MAIN = 3;
  public final static int LEFT_MOTOR_SLAVE = 2;
  public final static int RIGHT_MOTOR_SLAVE = 1;  
  
  // Shooter Motors
  public final static int SHOOTER_SLAVE_MOTOR = 5;
  public final static int SHOOTER_MOTOR = 6;

  // Telescope-Climb Motor
  public final static int CLIMBER_MOTOR = 13;
  public final static int TELESCOPE_MOTOR = 7;
  
  // Motor port used for wheel-of-fortune on control panel
  public final static int CONTROL_PANEL_WHEEL = 8;
  
  // Intake Motors
  public final static int INTAKE_ROTATOR_SLAVE = 9;
  public final static int INTAKE_ROTATOR = 10;
  public final static int INTAKE_SPINNER = 11;
  
  // Conveyor Motors
  public final static int CONVEYOR_BOTTOM = 12;
  public final static int CONVEYOR_TOP = 14;


  // Digital IO Sensors
  public final static int SHOOTER_SENSOR_LOW_CONVEYOR = 0;
  public final static int SHOOTER_SENSOR_READY = 1;
  public final static int SHOOTER_SENSOR_EJECT = 2;

  // Limit switch that detects when intake is all the way 'up'
  public final static int INTAKE_HOME_SENSOR = 5; // TODO Use correct DIO channel!
  
  // PWM port for LED Strip
  public static final int LED_STRIP = 8;

  // PCM ports used for solenoids
  public static final int HOOD_ADJUST = 4;
  public static final int INTAKE_AGITATOR = 5;
  public static final int GEAR_SOLENOID = 6;
  public static final int CONTROL_PANEL_SOLENOID = 7;
}
