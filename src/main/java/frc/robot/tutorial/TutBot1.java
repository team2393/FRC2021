package frc.robot.tutorial;

import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj.RobotController;
import frc.robot.BasicRobot;

/** Turns LED on DIO channel 8 on/off.
 * 
 *  <p>Try on roboRIO but may also try with
 *  View, Command Palette ..., "Simulate Robot on Desktop"
 */
public class TutBot1 extends BasicRobot
{
    private DigitalOutput led = new DigitalOutput(8);

    @Override
    public void teleopPeriodic()
    {
        // Turn LED on via USER button
        led.set( RobotController.getUserButton() );
    }

    @Override
    public void autonomousPeriodic()
    {
        // Turn LED on for even multiples of 500 ms
        led.set( (System.currentTimeMillis() / 500) % 2 == 0  );
    }
}
