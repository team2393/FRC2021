/*----------------------------------------------------------------------------*/
/* Copyright (c) 2020 FIRST Team 2393. All Rights Reserved.                   */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.recharge;

import java.io.File;

import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import frc.robot.BasicRobot;
import frc.robot.recharge.auto.ApplySettings;
import frc.robot.recharge.auto.AutonomousBuilder;
import frc.robot.recharge.climb.ClimbIdle;
import frc.robot.recharge.climb.Climber;
import frc.robot.recharge.climb.ControlClimber;
import frc.robot.recharge.ctrlpanel.ControlWheel;
import frc.robot.recharge.ctrlpanel.ExtendControlWheel;
import frc.robot.recharge.ctrlpanel.ManualWheelSpeed;
import frc.robot.recharge.ctrlpanel.RetractControlWheel;
import frc.robot.recharge.ctrlpanel.RotateToColor;
import frc.robot.recharge.ctrlpanel.RotateWheel;
import frc.robot.recharge.drivetrain.AutoShift;
import frc.robot.recharge.drivetrain.DriveByJoystick;
import frc.robot.recharge.drivetrain.DriveTrain;
import frc.robot.recharge.drivetrain.Reset;
import frc.robot.recharge.drivetrain.RotateToTarget;
import frc.robot.recharge.shooter.Eject;
import frc.robot.recharge.shooter.Hood;
import frc.robot.recharge.shooter.Intake;
import frc.robot.recharge.shooter.IntakeDown;
import frc.robot.recharge.shooter.IntakeUp;
import frc.robot.recharge.shooter.PowerCellAccelerator;
import frc.robot.recharge.shooter.ShooterIdle;

/**
 * Robot for 'Infinite Recharge' - R!$E2geTHeR#2020
 */
public class Enterprise extends BasicRobot
{ 
  private final DriveTrain drive_train = new DriveTrain();

  // Commands that require the drive train, i.e. starting any of these commands
  // will cancel whatever else was running and required the drive train
  private final CommandBase drive_by_joystick = new DriveByJoystick(drive_train);
  /** Most recent drive mode, either drive_by_joystick or heading_hold */
  private CommandBase drive_mode = drive_by_joystick;
  // After align_on_target, return to current drive_mode
  private final CommandBase align_on_target = new RotateToTarget(drive_train);
  private final CommandBase reset_drivetrain = new Reset(drive_train);
  
  private final Rumble rumble = new Rumble();
  
  // Shift commands can run concurrently with other commands that require the
  // drive train
  private final CommandBase shift_low = new InstantCommand(() -> drive_train.setGear(false));
  private final CommandBase shift_high = new InstantCommand(() -> drive_train.setGear(true));
  private final CommandBase auto_shift = new AutoShift(drive_train);

  // Ball handling
  private final Intake intake = new Intake();
  private final CommandBase intake_up = new IntakeUp(intake);
  private final CommandBase intake_down = new IntakeDown(intake);

  private final PowerCellAccelerator pca = new PowerCellAccelerator();
  private final CommandBase shooter_idle = new ShooterIdle(pca);
  private final CommandBase eject = new Eject(pca);
  
  private final Hood hood = new Hood();

  private final CommandBase hood_up = new InstantCommand(() -> hood.set(true));
  private final CommandBase hood_down = new InstantCommand(() -> hood.set(false));

  private final ControlWheel fortune = new ControlWheel();
  private final CommandBase extend_wheel = new ExtendControlWheel(fortune);
  private final CommandBase retract_wheel = new RetractControlWheel(fortune);
  private final CommandBase manual_wheel = new ManualWheelSpeed(fortune);
  private final CommandBase rotate_wheel = new RotateWheel(fortune, 3);
  private final CommandBase rotate_to_color = new RotateToColor(fortune);

  private Climber climber = new Climber();
  private CommandBase climb_idle = new ClimbIdle(climber);
  private CommandBase control_climb = new ControlClimber(climber);

  // private final LEDStrip led_strip = new LEDStrip();

  // Settings for different scenarios
  private final CommandBase near_settings = new ApplySettings("near.txt");
  private final CommandBase mid_settings = new ApplySettings("mid.txt");
  private final CommandBase far_settings = new ApplySettings("far.txt");
    
  private final SendableChooser<Command> auto_commands = new SendableChooser<>();

  // Teleop modes
  private enum TeleopMode
  {
    Drive,
    Climb,
    Wheel,
    SetUp
  };

  private TeleopMode teleop_mode = TeleopMode.Drive;

  @Override
  public void robotInit()
  {
    super.robotInit();

    // pcm.clearAllPCMStickyFaults();

    // Place some commands on dashboard
    SmartDashboard.putData("Auto Shift", auto_shift);

    SmartDashboard.putData("Reset Drive", reset_drivetrain);
    
    // SmartDashboard.putData("Heading Hold", heading_hold);
    // SmartDashboard.putData("Drive by Joystick", drive_by_joystick);
    
    // SmartDashboard.putData("Intake Up", intake_up);
    // SmartDashboard.putData("Intake Down", intake_down);

    SmartDashboard.putData("Setup Mode", new InstantCommand(()-> teleop_mode = TeleopMode.SetUp));

    SmartDashboard.setDefaultNumber("Teleop Hood Setpoint", -1);

    SmartDashboard.setDefaultNumber("Shooter RPM", PowerCellAccelerator.SHOOTER_RPM);

    // Auto options: Start with fixed options
    auto_commands.setDefaultOption("Nothing", new PrintCommand("Doing nothing"));
    try
    {
      // Add moves from auto.txt
      final File auto_file = new File(Filesystem.getDeployDirectory(), "auto.txt");
      for (CommandBase moves : AutonomousBuilder.read(auto_file, drive_train, intake, pca, hood))
        auto_commands.addOption(moves.getName(), moves);
    }
    catch (Exception ex)
    {
      System.out.println("Error in auto.txt:");
      ex.printStackTrace();
      System.out.println("========================\n\n\n");

      Timer.delay(10.0);
    }
    SmartDashboard.putData("Autonomous", auto_commands);

    // Allow selecting settings (more via buttons in robotPeriodic)
    SmartDashboard.putData("Viewable Settings", new ApplySettings("viewable.txt"));
    far_settings.schedule();
  }
  
  @Override
  public void disabledInit()
  {
    super.disabledInit();
    drive_train.lock(false);
    hood.set(false);
  }
  
  @Override
  public void robotPeriodic()
  {
    super.robotPeriodic();
    
    intake.unjam(OI.getUnjam());

    PowerCellAccelerator.SHOOTER_RPM = SmartDashboard.getNumber("Shooter RPM", PowerCellAccelerator.SHOOTER_RPM);
    SmartDashboard.putString("Teleop Mode", teleop_mode.toString());
    
    // Allow selecting settings for different scenarios
    if (OI.selectNear())
      near_settings.schedule();
    else if (OI.selectMid())
      mid_settings.schedule();
    else if (OI.selectFar())
      far_settings.schedule();
  }
  
  @Override
  public void teleopInit()
  {
    super.teleopInit();
    intake.resetToStartPosition(); //TODO Don't use in competition
 
    drive_train.lock(true);
    OI.reset();

    // auto_shift.schedule();
    drive_mode.schedule();
    pca.setState(PowerCellAccelerator.State.LOAD);
  }
  
  @Override
  public void teleopPeriodic()
  {
    if (teleop_mode == TeleopMode.Drive)
      teleop_drive();
    else if (teleop_mode == TeleopMode.Climb)
      teleop_climb();
    else if (teleop_mode == TeleopMode.Wheel)
      teleop_wheel();
    else if (teleop_mode == TeleopMode.SetUp)
      teleop_setup();
  }

  private void teleop_drive()
  {
    if (OI.selectClimbMode())
    {
      teleop_mode = TeleopMode.Climb;
      return;
    }
    else if (OI.selectWheelMode())
    {
      teleop_mode = TeleopMode.Wheel;
      extend_wheel.schedule();
      return;
    }

    // Disable climb control
    climb_idle.schedule();

    if (OI.getUnjam())
      pca.setState(PowerCellAccelerator.State.UN_JAM);
    else if (pca.getState() == PowerCellAccelerator.State.UN_JAM)
     pca.setState(PowerCellAccelerator.State.LOAD);
    
    // TODO Toggle hood solenoid with buttonboard
    if (OI.toggleHood())
      if (hood.getHoodPosition())
        hood_up.schedule();
      else
        hood_down.schedule();

    if (! auto_shift.isScheduled())
    {
      // Manual shifting
      if (OI.isLowGearRequested())
        shift_low.schedule();
      else if (OI.isHighGearRequested())
        shift_high.schedule();
    }
    
    // Toggle between drive_by_joystick and heading_hold
    OI.force_low_speed = false;

    if (OI.isIntakeTogglePressed())
      if (intake.getAngle() >= 20)
        intake_down.schedule();
      else  
        intake_up.schedule();

    // Holding the 'shoot' button starts or re-starts the command to shoot one ball.
    if (OI.isShootHeld())
      eject.schedule();

    // Align on target?
    if (OI.isAlignOnTargetHeld())
        align_on_target.schedule();
    else
    {
        // Re-enable original drive mode, which cancels alignment
        drive_mode.schedule();
    }
  }

  private void teleop_climb()
  {
    if (OI.selectDriveMode())
    {
      teleop_mode = TeleopMode.Drive;
      return;
    }
    else if (OI.selectWheelMode())
    {
      teleop_mode = TeleopMode.Wheel;
      extend_wheel.schedule();
      return;
    }
    intake_up.schedule();
    shooter_idle.schedule();

    auto_shift.cancel();
    shift_low.schedule();
    OI.force_low_speed = true;
    drive_by_joystick.schedule();
    
    control_climb.schedule();
  }

  private void teleop_wheel()
  {
    if (OI.selectDriveMode())
    {
      teleop_mode = TeleopMode.Drive;
      retract_wheel.schedule();
      return;
    }
    else if (OI.selectClimbMode())
    {
      teleop_mode = TeleopMode.Climb;
      retract_wheel.schedule();
      return;
    }
    intake_up.schedule();
    shooter_idle.schedule();

    auto_shift.cancel();
    shift_low.schedule();
    OI.force_low_speed = true;
    drive_by_joystick.schedule();
    
    if (OI.isAutorotateWheelRequested())
      rotate_wheel.schedule();
    else if (OI.isRotateToColorRequested())
      rotate_to_color.schedule();

    if (!rotate_to_color.isScheduled()  &&   !rotate_wheel.isScheduled())
      manual_wheel.schedule();
  }

  private void teleop_setup()
  {
    drive_mode.cancel();
    intake.setRotatorMotor(OI.getSpeed());
    if (OI.selectDriveMode())
      teleop_mode = TeleopMode.Drive;
  }

  @Override
  public void autonomousInit()
  {
    super.autonomousInit();

    OI.reset();
    SmartDashboard.putNumber("Teleop Hood Setpoint", -1);
    hood.set(false);
    intake.resetToStartPosition();
    drive_train.reset();
    drive_train.lock(true);
    pca.setState(PowerCellAccelerator.State.LOAD);
    
    // Run the selected command.
    auto_commands.getSelected().schedule();
  }

  @Override
  public void autonomousPeriodic()
  {
  }
}
