/*----------------------------------------------------------------------------*/
/* Copyright (c) 2020 FIRST Team 2393. All Rights Reserved.                   */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/
package frc.robot.recharge.auto;

import java.awt.BorderLayout;
import java.awt.Color;
import java.awt.Graphics;
import java.io.BufferedReader;
import java.io.FileInputStream;
import java.io.InputStreamReader;
import java.nio.file.Path;

import javax.swing.JFrame;
import javax.swing.JLabel;
import javax.swing.JPanel;
import javax.swing.SwingUtilities;

import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj.trajectory.Trajectory.State;

/** Plot trajectory on laptop
 * 
 *  Can be used by TrajectoryReader
 */
public class TrajectoryViewer
{
  private final Trajectory trajectory;
  private final double time_step;

  private class TrajectoryPlot extends JPanel
  {
    private static final long serialVersionUID = 1L;
    private double xmin, xmax, ymin, ymax, speedmax, traj_width, traj_height, scale;

    @Override
    protected void paintComponent(final Graphics g)
    {
      xmin = 0;
      xmax = 1;
      ymin = 0;
      ymax = 1;
      speedmax = 0;
      // Determine bounding box of trajectory
      for (State state : trajectory.getStates())
      {
        final double x = state.poseMeters.getTranslation().getX(),
                     y = state.poseMeters.getTranslation().getY();
        xmin = Math.min(xmin, x);
        xmax = Math.max(xmax, x);
        ymin = Math.min(ymin, y);
        ymax = Math.max(ymax, y);
        speedmax = Math.max(speedmax, Math.abs(state.velocityMetersPerSecond));
      }
      // Largest size (width or height) of trajectory
      traj_width = xmax - xmin;
      traj_height = ymax - ymin;
      final double traj_size = Math.max(traj_width, traj_height);
      // Scale to fit onto plot allowing for 10 pixels around edges
      final double plot_size = Math.min(getWidth(), getHeight()) - 20;
      scale = plot_size / traj_size;

      final double total_time = trajectory.getTotalTimeSeconds();
      // Draw all  states
      g.setColor(Color.BLUE);
      for (double time=0; time < total_time+1;  time += time_step)
        draw(g, trajectory.sample(time));
      // Draw start and end again in standout colors
      g.setColor(Color.GREEN);
      draw(g, trajectory.sample(0));
      g.setColor(Color.RED);
      draw(g, trajectory.sample(total_time));
    }

    private void draw(final Graphics g, final State state)
    {
      final int x = 10 + (int) Math.round((traj_height - state.poseMeters.getTranslation().getY() + ymin) * scale);
      final int y = 10 + (int) Math.round((traj_width  - state.poseMeters.getTranslation().getX()) * scale);
      g.fillOval(x-5, y-5, 10, 10);

      final double speed = state.velocityMetersPerSecond * 20 / speedmax;
      final int x1 = x - (int) Math.round(state.poseMeters.getRotation().getSin() * speed);
      final int y1 = y - (int) Math.round(state.poseMeters.getRotation().getCos() * speed);
      g.drawLine(x, y, x1, y1);
    }
  }

  public TrajectoryViewer(final Trajectory trajectory)
  {
    this(trajectory, 0.5);
  }

  public TrajectoryViewer(final Trajectory trajectory, final double time_step)
  {
    this.trajectory = trajectory;
    this.time_step = time_step;
    SwingUtilities.invokeLater(this::createAndShowPlot);
  }

  private void createAndShowPlot()
  {
    final JFrame frame = new JFrame("Trajectory Viewer");
    frame.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);

    frame.getContentPane().add(new JLabel(TrajectoryHelper.getEndInfo(trajectory)),
                        BorderLayout.NORTH);
    
    frame.getContentPane().add(new TrajectoryPlot(),
                               BorderLayout.CENTER);

    frame.getContentPane().add(new JLabel(String.format("Total time: %.2f seconds, max speed %.1f m/s",
                                                        trajectory.getTotalTimeSeconds(),
                                                        TrajectoryHelper.getMaxVelocity(trajectory))),
                               BorderLayout.SOUTH);
    frame.setBounds(10, 10, 600, 800);
    frame.setVisible(true);
  }

  public static void main(String[] args) throws Exception
  {
    // // Create demo trajectory
    // final TrajectoryConfig config = new TrajectoryConfig(1.0, 0.3);
    // final Pose2d start = new Pose2d(0.0, 0.0, Rotation2d.fromDegrees(0.0));
    // Translation2d pos = start.getTranslation();
    
    // final List<Translation2d> waypoints = new ArrayList<>();
    // pos = pos.plus(new Translation2d(1, 0));
    // waypoints.add(pos);
    
    // pos = pos.plus(new Translation2d(1, 1));
    // waypoints.add(pos);
    
    // pos = pos.plus(new Translation2d(1, 0));
    // final Pose2d end = new Pose2d(pos, Rotation2d.fromDegrees(0.0));
    // final Trajectory trajectory = TrajectoryGenerator.generateTrajectory(start, waypoints, end, config);

    // Trajectory trajectory = TrajectoryUtil.fromPathweaverJson(Path.of("src/main/deploy/output/1MeterSwerve.wpilib.json"));
    // trajectory = trajectory.relativeTo(trajectory.sample(0).poseMeters);
    // trajectory = TrajectoryHelper.reverse(trajectory);

    // Open file
    final BufferedReader file = new BufferedReader(new InputStreamReader(new FileInputStream("src/main/deploy/simu.txt")));
    // Find desire Auto XXXX..
    String line;
    for (line = file.readLine();
         line != null;
         line = file.readLine())
         if (line.contains("Auto Barrel"))
            break;
    line = file.readLine();
    if (! line.contains("Poses"))
        throw new Exception("Expect 'Poses', got " + line); 
    Trajectory trajectory = TrajectoryReader.readPoses(file, false);

    // Show it
    new TrajectoryViewer(trajectory);
  }
}