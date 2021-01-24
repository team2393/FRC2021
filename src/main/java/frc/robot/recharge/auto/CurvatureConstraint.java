/*----------------------------------------------------------------------------*/
/* Copyright (c) 2020 FIRST Team 2393. All Rights Reserved.                   */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/
package frc.robot.recharge.auto;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.trajectory.constraint.TrajectoryConstraint;

/** A TrajectoryConstraint that limits the rotation */
public class CurvatureConstraint implements TrajectoryConstraint
{
  private static final MinMax ALLOW_ANY_ACCELERATION = new MinMax();
  private final double max_rad_per_second;

  /** @param max_degree_per_second How fast can we rotate? E.g. 45 degree/sec */
  public CurvatureConstraint(final double max_degree_per_second)
  {
    max_rad_per_second = Math.toRadians(max_degree_per_second);
  }

  @Override
  public double getMaxVelocityMetersPerSecond(final Pose2d poseMeters, final double curvatureRadPerMeter,
                                              final double velocityMetersPerSecond)
  {
    // How fast would we be rotating?
    final double rad_per_second = velocityMetersPerSecond * curvatureRadPerMeter;

    // Slow enough?
    if (Math.abs(rad_per_second) < max_rad_per_second)
      return velocityMetersPerSecond;

    // Too fast!
    // Compute how fast we may drive to hit max_rad_per_second,
    // then return that with the same sign as the originally planned speed
    return Math.signum(velocityMetersPerSecond) * Math.abs(max_rad_per_second / curvatureRadPerMeter);      
  }

  @Override
  public MinMax getMinMaxAccelerationMetersPerSecondSq(final Pose2d poseMeters, final double curvatureRadPerMeter,
                                                        final double velocityMetersPerSecond)
  {
    // Don't limit acceleration any further
    return ALLOW_ANY_ACCELERATION;
  }
}