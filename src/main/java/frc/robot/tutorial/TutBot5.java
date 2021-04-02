package frc.robot.tutorial;

import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.BasicRobot;

/** Control both LED and servo using commands */
public class TutBot5 extends BasicRobot
{
    private DigitalOutput white_led = new DigitalOutput(8);
    private DigitalOutput red_led = new DigitalOutput(5);

    private CommandBase level = new AutoLevelCommand();


    @Override
    public void autonomousInit()
    {
        super.autonomousInit();

        SmartDashboard.putData("AutoLevel", level);
        SmartDashboard.putData("LED ON", new LEDOnCommand(white_led));

        System.out.println("Starting LED commands...");

        new SequentialCommandGroup(
            new BlinkLEDCommand(white_led, 200, 5),

            new ParallelCommandGroup(
                new BlinkLEDCommand(white_led, 100, 5),
                new BlinkLEDCommand(red_led, 100, 5)
            ),

            new BlinkLEDCommand(red_led, 1000, 10),
            new BlinkLEDCommand(white_led, 50, 3)
        ).schedule();

    }

    @Override
    public void autonomousPeriodic()
    {
    }
}
