/*----------------------------------------------------------------------------*/
/* Copyright (c) 2020 FIRST Team 2393. All Rights Reserved.                   */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/
package frc.robot.recharge.auto;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Transform2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.trajectory.Trajectory.State;
import frc.robot.recharge.drivetrain.DriveTrain;

/** Helpers for dealing with Trajectory */
public class TrajectoryHelper
{
  /** @param trajectory Trajectory
   *  @return End state
   */
  public static State getEnd(final Trajectory trajectory)
  {
    final List<State> states = trajectory.getStates();
    return states.get(states.size()-1);
  }

  /** @param trajectory Trajectory
   *  @return End pose
   */
  public static Pose2d getEndPose(final Trajectory trajectory)
  {
    final List<State> states = trajectory.getStates();
    return states.get(states.size()-1).poseMeters;
  }

  /** @param state One state along a Trajectory
   *  @return "X=..., Y=..., ..."
   */
  public static String getInfo(final State state)
  {
    return String.format("X=%.2f m, Y=%.2f m, Heading=%.1f degrees",
                         state.poseMeters.getTranslation().getX(),
                         state.poseMeters.getTranslation().getY(),
                         state.poseMeters.getRotation().getDegrees());
  }

  /** @param trajectory Trajectory;
   *  @return Info about end point
   */
  public static String getEndInfo(final Trajectory trajectory)
  {
    return "End: " + getInfo(getEnd(trajectory));
  }

  /** Revert a trajectory
   * 
   *  Change end point to start point,
   *  next-to-last into second point.
   *  Inverts direction of velocities etc.
   * 
   *  An original trajectory that moves 0 -> 1
   *  turns into one that moves 0 -> -1,
   * 
   *  Meant to turn existing 'forward' trajectory
   *  into one that's used in 'reverse', going
   *  backwards.
   * 
   * @param original Trajectory
   * @return Reversed trajectory
   */
  public static Trajectory reverse(final Trajectory original)
  {
    final List<State> orig_states = original.getStates();
    final List<State> reversed_states = new ArrayList<>();

    final State end = orig_states.get(orig_states.size()-1);
    final double end_x = end.poseMeters.getTranslation().getX();
    final double end_y = end.poseMeters.getTranslation().getY();
    final double duration = original.getTotalTimeSeconds();

    // Run through original states in reverse order
    for (int i=orig_states.size()-1;  i>=0;  --i)
    {
      final State orig = orig_states.get(i);
      reversed_states.add(new State(duration - orig.timeSeconds,
                                    -orig.velocityMetersPerSecond,
                                    orig.accelerationMetersPerSecondSq,
                                     new Pose2d(new Translation2d(orig.poseMeters.getTranslation().getX() - end_x,
                                                                  orig.poseMeters.getTranslation().getY() - end_y),
                                                orig.poseMeters.getRotation()),
                                    orig.curvatureRadPerMeter));
    }
    return new Trajectory(reversed_states);
  }

  /** Re-locate and rotate a trajectory
   *  @param Original trajectory
   *  @param start Startpoint and heading
   *  @return Transformed trajectory where initial state is 'start'
   */
  public static Trajectory makeTrajectoryStartAt(final Trajectory trajectory, final Pose2d start)
  {
    // The relativeTo() methods in Pose2d and Trajectory are basically
    // a "substract" operation for X, Y and the rotation angle.
    // Assume a trajectory that moves from "5" to "6", and start is at "2".
    // The offset becomes "5" - "2" = "3"
    final Transform2d offset = start.minus(trajectory.getInitialPose());
    // Computing the "5 -> 6" trajectory relative to "3" becomes "2 -> 3".
    // So we get a trajectory that moves by "1" unit, starting at "2".
    return trajectory.transformBy(offset);
  }

  public static void main(String[] args) throws Exception
  {
    // Trajectory that moves 1m forward, anchored at 0
    Trajectory forward1 = TrajectoryGenerator.generateTrajectory(
          new Pose2d(0, 0, Rotation2d.fromDegrees(0)),
          Collections.emptyList(),
          new Pose2d(1, 0, Rotation2d.fromDegrees(0)),
          DriveTrain.trajectory_config);

    // Trajectory that moves 1m forward, but anchored at 10
    Trajectory forward2 = TrajectoryGenerator.generateTrajectory(
          new Pose2d(10, 0, Rotation2d.fromDegrees(0)),
          Collections.emptyList(),
          new Pose2d(11, 0, Rotation2d.fromDegrees(0)),
          DriveTrain.trajectory_config);
    
    // Trajectory that rotates right while moving 1m forward, anchored at 0
    Trajectory right = TrajectoryGenerator.generateTrajectory(
          new Pose2d(0, 0, Rotation2d.fromDegrees(0)),
          Collections.emptyList(),
          new Pose2d(1, -1, Rotation2d.fromDegrees(-90)),
          DriveTrain.trajectory_config);

    Pose2d nominal = new Pose2d(0, 0, Rotation2d.fromDegrees(0));
    Trajectory xform;
    System.out.println("Start: "+ nominal);
    xform = makeTrajectoryStartAt(forward1, nominal);
    xform.getStates().forEach(System.out::println);
    nominal = getEndPose(xform);
    System.out.println("1m: "+ nominal);
    if (! nominal.equals(new Pose2d(1, 0, Rotation2d.fromDegrees(0))))
      throw new Exception("Ooops");

    xform = makeTrajectoryStartAt(forward2, nominal);
    xform.getStates().forEach(System.out::println);
    nominal = getEndPose(xform);
    System.out.println("2m: "+ nominal);
    if (! nominal.equals(new Pose2d(2, 0, Rotation2d.fromDegrees(0))))
      throw new Exception("Ooops");

    xform = makeTrajectoryStartAt(right, nominal);
    xform.getStates().forEach(System.out::println);
    nominal = getEndPose(xform);
    System.out.println("3, -1, -90: "+ nominal);
    if (! nominal.equals(new Pose2d(3, -1, Rotation2d.fromDegrees(-90))))
      throw new Exception("Ooops");

    xform = makeTrajectoryStartAt(right, nominal);
    xform.getStates().forEach(System.out::println);
    nominal = getEndPose(xform);
    System.out.println("2, -2, -180: "+ nominal);
    if (! nominal.equals(new Pose2d(2, -2, Rotation2d.fromDegrees(-180))))
      throw new Exception("Ooops");
  }
}