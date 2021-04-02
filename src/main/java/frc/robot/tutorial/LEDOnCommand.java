package frc.robot.tutorial;

import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj2.command.InstantCommand;

/** Command that turns LED on */
public class LEDOnCommand extends InstantCommand
{
    private DigitalOutput led;

    /**
     * @param led_to_use The LED to use
     */
    public LEDOnCommand(DigitalOutput led_to_use)
    {
        led = led_to_use;
    }

    @Override
    public void initialize()
    {
        led.set(true);
    }
}
