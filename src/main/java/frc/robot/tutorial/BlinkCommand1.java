package frc.robot.tutorial;

import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj2.command.CommandBase;

/** Command that 'blinks' an output at some period */
public class BlinkCommand1 extends CommandBase
{
    private final DigitalOutput output;
    private final long period;
    
    /** @param output DigitalOutput to use
     *  @param period Blink period in millisec
     */
    public BlinkCommand1(DigitalOutput output, long period)
    {
        this.output = output;
        this.period = period;
    }

    @Override
    public void execute()
    {
        output.set( (System.currentTimeMillis() / period) % 2 == 0);
    }
}
