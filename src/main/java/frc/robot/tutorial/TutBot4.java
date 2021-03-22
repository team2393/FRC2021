package frc.robot.tutorial;

import edu.wpi.first.wpilibj.BuiltInAccelerometer;
import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.interfaces.Accelerometer;
import frc.robot.BasicRobot;

/** Control both LED and servo */
public class TutBot4 extends BasicRobot
{
    private DigitalOutput led = new DigitalOutput(8);
    private Servo rotator = new Servo(7);
    private Accelerometer accel = new BuiltInAccelerometer();

    private double smoothed_angle = 0.0;

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

        // Level servo
        double angle = Math.toDegrees(Math.atan2(accel.getY(), accel.getZ()) );

        System.out.printf("X= %4.1f, Y= %4.1f, Z= %4.1f  -> angle %4.0f\n",
                          accel.getX(), accel.getY(), accel.getZ(),
                          angle
                          );
        
        smoothed_angle = 0.9 * smoothed_angle + 0.1 * angle;
     
        rotator.setAngle(90 - smoothed_angle);
    }
}
