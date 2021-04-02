package frc.robot.tutorial;

import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj2.command.CommandBase;

/** Command to blink given LED at specified period with timeout */
public class BlinkLEDCommand extends CommandBase
{
    private DigitalOutput led;
    private long millis, run_seconds, end_millisecs;

    /**
     * @param led_to_use The LED to use
     * @param millis Blink period in milliseconds
     * @param run_seconds How long to do this
     */
    public BlinkLEDCommand(DigitalOutput led_to_use,
                           int millis,
                           int run_seconds)
    {
        led = led_to_use;
        this.millis = millis;
        this.run_seconds = run_seconds;
    }

    @Override
    public void initialize()
    {
        end_millisecs = System.currentTimeMillis() + run_seconds * 1000;
    }

    @Override
    public void execute()
    {
        // Turn LED on for even multiples of xxxx ms
        led.set( (System.currentTimeMillis() / millis) % 2 == 0  );
    }

    @Override
    public boolean isFinished()
    {
        return System.currentTimeMillis() >= end_millisecs;
    }

    @Override
    public void end(boolean interrupted)
    {
        led.set(false);
    }
}
