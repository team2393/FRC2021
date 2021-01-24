/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST Team 2393. All Rights Reserved.                   */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.demo.commands;

import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.BasicRobot;

/** Demo of 'new' commands
 *
 *  Blinks some output in auto
 */
public class CommandRobot extends BasicRobot
{
    // In larger robot project, the port numbers should be in a RobotMap..
    private final DigitalOutput led = new DigitalOutput(1);

    // .. and the operator interface details should be in an OI class...
    private final XboxController joystick = new XboxController(0);
    private final JoystickButton test1 = new JoystickButton(joystick, XboxController.Button.kA.value);
    private final JoystickButton test2 = new JoystickButton(joystick, XboxController.Button.kB.value);

    // Our command that blinks
    private final Command blink = new Blink(led, 500);

    // Same functionality, but assembled from basic commands
    private final Command led_on = new InstantCommand(() -> led.set(true));
    private final Command led_off = new InstantCommand(() -> led.set(false));
    private final Command blink_seq = new SequentialCommandGroup(led_on,
                                                                 new WaitCommand(0.5),
                                                                 led_off,
                                                                 new WaitCommand(0.5))
    {
        @Override
        public void end(boolean interrupted)
        {
            // Always make sure that LED is off
            led.set(false);
            // .. then do whatever SequentialCommandGroup does to end()
            super.end(interrupted);
        }
    };

    @Override
    public void robotInit()
    {
        super.robotInit();
        System.out.print("Hold button A or B to blink on DIO 1");

        // Bind Joystick to commands (used in teleop)
        // Note subtle difference:
        // Pressing the button starts our 'Blink' command,
        // which then runs forever until releasing the button ends it.
        // On end(), the LED will be turned off.
        test1.whileHeld(blink);
        // Pressing the button starts the blink_seq, which blinks once,
        // then ends, and keeps getting restarted while the button is held.
        // When the button is released, end() will turn the LED off.
        test2.whileHeld(blink_seq);
    }

    @Override
    public void autonomousInit()
    {
        super.autonomousInit();
        blink.schedule();
    }
}
