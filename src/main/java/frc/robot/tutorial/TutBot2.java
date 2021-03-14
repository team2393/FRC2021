package frc.robot.tutorial;

import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Servo;
import frc.robot.BasicRobot;

/** Servo: Rotate via USER button or autonomously */
public class TutBot2 extends BasicRobot
{
    private Servo rotator = new Servo(7);

    @Override
    public void teleopPeriodic()
    {
        if ( RobotController.getUserButton() )
            rotator.setAngle(90);
        else
            rotator.setAngle(45);
    }
        
    @Override
    public void autonomousPeriodic()
    {
        double angle;
        
        // Cycle angle between 45 and 90 every 3 seconds
        if ( (System.currentTimeMillis() / 3000) % 2 == 0 )
            angle = 45;
        else
            angle = 90;

        rotator.setAngle(angle);
    }
}
