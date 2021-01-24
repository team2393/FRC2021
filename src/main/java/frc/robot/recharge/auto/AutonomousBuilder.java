/*----------------------------------------------------------------------------*/
/* Copyright (c) 2020 FIRST Team 2393. All Rights Reserved.                   */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/
package frc.robot.recharge.auto;

import java.io.BufferedReader;
import java.io.File;
import java.io.FileReader;
import java.util.ArrayList;
import java.util.List;
import java.util.Scanner;

import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.Trajectory.State;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.recharge.drivetrain.DriveTrain;
import frc.robot.recharge.drivetrain.RotateToTarget;
import frc.robot.recharge.drivetrain.TurnToHeading;
import frc.robot.recharge.shooter.Eject;
import frc.robot.recharge.shooter.Hood;
import frc.robot.recharge.shooter.HoodDown;
import frc.robot.recharge.shooter.HoodUp;
import frc.robot.recharge.shooter.Intake;
import frc.robot.recharge.shooter.IntakeDown;
import frc.robot.recharge.shooter.IntakeMid;
import frc.robot.recharge.shooter.IntakeUp;
import frc.robot.recharge.shooter.PowerCellAccelerator;

/** Build auto commands from a file */
public class AutonomousBuilder
{
  /** @param filename File to read
   *  @param trajectory_command Function that turns trajectory into command
   *  @throws Exception on error
   */
  public static List<SequentialCommandGroup> read(final File filename,
                                                  final DriveTrain drive_train,
                                                  final Intake intake,
                                                  final PowerCellAccelerator pca,
                                                  final Hood hood) throws Exception
  {
    final BufferedReader file = new BufferedReader(new FileReader(filename));
    final List<SequentialCommandGroup> autos = new ArrayList<>();
    SequentialCommandGroup current_auto = null;

    // Track where we think the robot should be.
    // At the end of a trajectory, it's set to the final pose
    // and the next trajectory then starts there.
    // The trajectory follower can thus correct when it compares
    // to encoder readings.
    Pose2d nominal = null;

    String line;
    while ((line = file.readLine()) != null)
    {
      // Skip empty lines and comments
      if (line.isBlank()  ||  line.startsWith("#"))
        continue;
  
      try
      (
        Scanner scanner = new Scanner(line);
      )
      {
        final String command = scanner.next();
        if (command.equals("Auto"))
        { // Auto Name-of-this-sequence:
          // Start new auto
          current_auto = new SequentialCommandGroup();
          current_auto.setName(scanner.nextLine());
          autos.add(current_auto);
          // Initial position at start of autonomous
          nominal = new Pose2d();
          System.out.println("Reading Auto '" + current_auto.getName() + "''");
        }
        else if (command.equals("Trajectory") ||
                command.equals("ReverseTrajectory"))
        { // Trajectory:
          // Read trajectory info
          Trajectory trajectory = TrajectoryReader.readPoints(file, command.equals("ReverseTrajectory"));
          // Make it start at the assumed position
          trajectory = TrajectoryHelper.makeTrajectoryStartAt(trajectory,  nominal);
          // .. and then we expect to be where the trajectory ends
          nominal = TrajectoryHelper.getEndPose(trajectory);

          // Turn into command, which may be a 'print' or a 'ramsete' that follows it
          current_auto.addCommands(drive_train.createRamsete(trajectory));
          System.out.format("Added %s (%.1f seconds)\n", command, trajectory.getTotalTimeSeconds());
        }
        else if (command.equals("Poses") ||
                command.equals("ReversePoses"))
        { // Trajectory:
          // Read trajectory info
          Trajectory trajectory = TrajectoryReader.readPoses(file, command.equals("ReversePoses"));
          // Make it start at the assumed position
          trajectory = TrajectoryHelper.makeTrajectoryStartAt(trajectory,  nominal);
          // .. and then we expect to be where the trajectory ends
          nominal = TrajectoryHelper.getEndPose(trajectory);

          // Turn into command, which may be a 'print' or a 'ramsete' that follows it
          current_auto.addCommands(drive_train.createRamsete(trajectory));
          System.out.format("Added %s (%.1f seconds)\n", command, trajectory.getTotalTimeSeconds());
        }
        else if (command.equals("PathWeaver")  ||
                command.equals("ReverseWeaver"))
        { // Read trajectory from PathWeaver file
          final File pwfile = new File(Filesystem.getDeployDirectory(), scanner.next());
          System.out.println(pwfile);
          Trajectory trajectory = TrajectoryReader.readPath(pwfile);
          if (command.equals("ReverseWeaver"))
            trajectory = TrajectoryHelper.reverse(trajectory);
          // Make it start at the assumed position
          System.out.println("Move start point to " + nominal);
          trajectory = TrajectoryHelper.makeTrajectoryStartAt(trajectory,  nominal);
          // .. and then we expect to be where the trajectory ends
          nominal = TrajectoryHelper.getEndPose(trajectory);

          for (State state : trajectory.getStates())
            System.out.println(state);

          current_auto.addCommands(drive_train.createRamsete(trajectory));
          System.out.format("Added %s (%.1f seconds)\n", command, trajectory.getTotalTimeSeconds());
        }
        else if(command.equals("RotateToTarget"))
            current_auto.addCommands(new RotateToTarget(drive_train));
        else if(command.equals("TurnToHeading"))
        {
          final double heading = scanner.nextDouble();
          current_auto.addCommands(new TurnToHeading(drive_train, heading));
        }
        else if(command.equals("Wait"))
        {
          final double time = scanner.nextDouble();
          current_auto.addCommands(new WaitCommand(time));
        }
        else if(command.equals("IntakeUp"))
        current_auto.addCommands(new IntakeUp(intake)); 
        else if(command.equals("IntakeDown"))
        current_auto.addCommands(new IntakeDown(intake)); 
        else if(command.equals("IntakeMid"))
        current_auto.addCommands(new IntakeMid(intake)); 
        else if(command.equals("Hood"))
        {
          final boolean hood_position = scanner.nextBoolean();
          if (hood_position)
            current_auto.addCommands(new HoodUp(hood));
          else
            current_auto.addCommands(new HoodDown(hood));
        }
        else if(command.equals("Shoot"))
        {
          int times = 1;
          if (scanner.hasNextInt())
            times = scanner.nextInt();
          if (times < 1   ||  times > 5)
            throw new Exception("Invalid shot count: " + line);
          while (times-- > 0)
            current_auto.addCommands(new Eject(pca));
        }
        else
          throw new Exception("Unknown autonomouse command: " + line);
      }
    }

    return autos;
  }
}