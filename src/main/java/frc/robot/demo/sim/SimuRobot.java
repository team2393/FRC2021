/*----------------------------------------------------------------------------*/
/* Copyright (c) 2021 FIRST Team 2393. All Rights Reserved.                   */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/
package frc.robot.demo.sim;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.BasicRobot;

/** Demo for a robot with simulation.
 * 
 *  Run via
 *  View, Command, WPILib: Simulate...,
 *  then pick the 'halsim_gui.dll'.
 * 
 *  To stop, close the simulation GUI.
 */ 
public class SimuRobot extends BasicRobot
{
    private final DigitalInput limit = new DigitalInput(0);
    private final DigitalOutput horn = new DigitalOutput(1);
    private final WPI_TalonFX motor = new WPI_TalonFX(1);
    
    // Falcon/Talon doesn't fully support simulation.
    // Turns out motor.getSelectedSensorPosition() keeps reporting 0 in simulation,
    // motor.setSelectedSensorPosition(sensorPos) is ignored,
    // so we need to track the simulated position ourself
    private double simulated_position = 0.0;
    
    @Override
    public void robotInit()
    {
        super.robotInit();

        motor.configFactoryDefault();
        motor.setNeutralMode(NeutralMode.Brake);
        motor.configOpenloopRamp(0.1);
        
        // Until every piece of hardware supports simulation,
        // it might be necessary to check at startup:
        if (RobotBase.isReal())
        {
            // Do what's only possible on real robot
        }    
        else
        {
            System.out.println("** Running in simulation **");
        }    
    }    

    @Override
    public void teleopPeriodic()
    {
        // 'limit' switch is usually closed, connecting the input to 5V, reading 1/true.
        // When the switch it hit, it opens, reading 0V/0/false.
        // 'horn' should turn on when the limit switch is hit.
        // Simulate the 'limit' input in simulation GUI and check 'horn' response
        horn.set( ! limit.get());

        final boolean high = (System.currentTimeMillis() / 3000) % 2 == 0;
        motor.set(ControlMode.PercentOutput, high ? 0.9 : -0.1);

        if (RobotBase.isReal())
            SmartDashboard.putNumber("Position", motor.getSelectedSensorPosition());
        else
            SmartDashboard.putNumber("Position", simulated_position);
    }

    // Code run whenever/only in simulation.
    // Seems to be called just _after_ teleopPeriodic()
    @Override
    public void simulationPeriodic()
    {
        // In simulation, the non-existing motor isn't actually turning,
        // so we need to simulate how fast it would go and update the simulated position.

        // Motor RPM per volt
        final double kV = 1000;
        // motor.getMotorOutputPercent() and motor.getMotorOutputVoltage() report 0 in simulation,
        // but motor.get() gives last commanded value.
        final double voltage = motor.get() * RobotController.getBatteryVoltage();
        // Estimated RPM
        final double rpm = voltage * kV;
        // Estimated steps that motor moves within one 'period'
        final double steps = rpm / 60.0 * getPeriod();
        simulated_position += steps;
    }
}
