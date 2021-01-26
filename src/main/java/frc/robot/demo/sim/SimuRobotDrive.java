/*----------------------------------------------------------------------------*/
/* Copyright (c) 2020 FIRST Team 2393. All Rights Reserved.                   */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/
package frc.robot.demo.sim;

import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.BasicRobot;
import frc.robot.recharge.drivetrain.DriveByJoystick;
import frc.robot.recharge.drivetrain.DriveTrain;

/** Demo for a robot with simulation.
 * 
 *  Run via
 *  View, Command, WPILib: Simulate...,
 *  then pick the 'halsim_gui.dll'.
 * 
 *  To stop, close the simulation GUI.
 */ 
public class SimuRobotDrive extends BasicRobot
{
    // 'Normal' robot components and commands
    private final DriveTrain drivetrain = new DriveTrain();
    private final Command joydrive = new DriveByJoystick(drivetrain);

    @Override
    public void teleopInit()
    {
        super.teleopInit();
        drivetrain.reset();
        joydrive.schedule();
    }
}
