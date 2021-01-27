/*----------------------------------------------------------------------------*/
/* Copyright (c) 2020 FIRST Team 2393. All Rights Reserved.                   */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/
package frc.robot.demo.sim;

import java.io.File;

import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import frc.robot.BasicRobot;
import frc.robot.recharge.auto.AutonomousBuilder;
import frc.robot.recharge.drivetrain.DriveByJoystick;
import frc.robot.recharge.drivetrain.DriveTrain;
import frc.robot.recharge.shooter.Hood;
import frc.robot.recharge.shooter.Intake;
import frc.robot.recharge.shooter.PowerCellAccelerator;

/**
 * Demo for a robot with simulation.
 * 
 * Run via View, Command, WPILib: Simulate..., then pick the 'halsim_gui.dll'.
 * 
 * To stop, close the simulation GUI.
 */
public class SimuRobotDrive extends BasicRobot
{
    // 'Normal' robot components and commands
    private final DriveTrain drive_train = new DriveTrain();
    private final Command joydrive = new DriveByJoystick(drive_train);

    private final Intake intake = null;
    private final PowerCellAccelerator pca = null;
    private final Hood hood = null;

    private final SendableChooser<Command> auto_commands = new SendableChooser<>();

    @Override
    public void robotInit()
    {
        super.robotInit();

        // Auto options: Start with fixed options
        auto_commands.setDefaultOption("Nothing", new PrintCommand("Doing nothing"));
        try
        {
            // Add moves from simu.txt instead of auto.txt
            // because not all commands (intake, ..) work at this time
            final File auto_file = new File(Filesystem.getDeployDirectory(), "simu.txt");
            for (CommandBase moves : AutonomousBuilder.read(auto_file, drive_train, intake, pca, hood))
            auto_commands.addOption(moves.getName(), moves);
        }
        catch (Exception ex)
        {
            System.out.println("Error in simu.txt:");
            ex.printStackTrace();
            System.out.println("========================\n\n\n");

            Timer.delay(10.0);
        }
        SmartDashboard.putData("Autonomous", auto_commands);
    }

    @Override
    public void teleopInit()
    {
        super.teleopInit();
        drive_train.reset();
        joydrive.schedule();
    }

    @Override
    public void autonomousInit()
    {
        super.autonomousInit();

        // Run the selected command.
        auto_commands.getSelected().schedule();
    }
}
