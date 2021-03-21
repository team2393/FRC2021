package frc.robot.tutorial;

import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.interfaces.Accelerometer;
import edu.wpi.first.wpilibj2.command.CommandBase;

/** Command that 'levels' something using a servo based
 *  on tilt angle measured via accelerometer
 */
public class LevelCommand extends CommandBase
{
    private final Servo rotator;
    private final Accelerometer accel;

    private double smoothed_angle = 0.0;
    
    /** @param rotator Servo to use
     *  @param accel Accelerometer to use
     */
    public LevelCommand(Servo rotator, Accelerometer accel)
    {
        this.rotator = rotator;
        this.accel = accel;
    }

    @Override
    public void initialize()
    {
        // XXX Reset smoothed angle?
        smoothed_angle = 0.0;
    }

    @Override
    public void execute()
    {
        double angle = Math.toDegrees(Math.atan2(accel.getY(), accel.getZ()) );

        System.out.printf("X= %4.1f, Y= %4.1f, Z= %4.1f  -> angle %4.0f\n",
                          accel.getX(), accel.getY(), accel.getZ(),
                          angle);
        
        // Smoothing: Use 90% of old value, 10% of new measurement
        // to slowly change from old to new.
        smoothed_angle = 0.9 * smoothed_angle + 0.1 * angle;
     
        // How exactly we move the servo (for example +-90 degree)
        // depends on how it's mounted on the robot, usually
        // requires some trial & error
        rotator.setAngle(90 - smoothed_angle);
    }
}
