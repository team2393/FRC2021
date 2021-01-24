/*----------------------------------------------------------------------------*/
/* Copyright (c) 2020 FIRST Team 2393. All Rights Reserved.                   */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.recharge.shooter;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;

/** Eject one power cell.
 *  Since ejector keeps running a little longer,
 *  rescheduling this command right away keeps
 *  the cells flowing.
 */
public class Eject extends CommandBase
{
  private final PowerCellAccelerator pca;
  private enum State
  {
    SPINUP,    // Spinning up, waiting for correct RPM
    EJECT,     // Trying feed a ball to ejector
    SUCCESS,   // Saw a ball shoot out
    TIMEOUT    // Give up, no ball seen flying out
  };
  private State state = State.SUCCESS;
  private final Timer spinup_timer = new Timer();
  private final Timer timeout_timer = new Timer();

  public Eject(final PowerCellAccelerator pca)
  {
    this.pca = pca;
    addRequirements(pca);
  }

  @Override
  public void initialize()
  {
    // Turn on the ejector, but don't actually eject, yet
    pca.enableLoad(true);
    pca.feedEjector(false);
    pca.eject(true);
    state = State.SPINUP;
    spinup_timer.reset();
    spinup_timer.start();
    timeout_timer.reset();
    System.out.println("EJECT: " + state);
  }

  @Override
  public void execute()
  {
    if (state == State.SPINUP)
    {
      double rpm = pca.getShooterRPM();
      // Once it's fast enough, SHOOT!!
      if (rpm >= PowerCellAccelerator.MINIMUM_RPM_FRACTION * PowerCellAccelerator.SHOOTER_RPM)
      {
        state = State.EJECT;
        spinup_timer.stop();
        System.out.println("EJECT: " + state + " at " + rpm + " RPMS after spinup of " + spinup_timer.get() + " seconds");
        timeout_timer.start();
      }
      else
      {
        // Not fast enough. If there's a ball ready, keep it there
        // System.out.println("EJECT: Low rpm " + rpm);
      }
    }
    
    // Are we shooting? If so, move a ball out
    if (state == State.EJECT)
    {
      pca.feedEjector(true);
      // Ideally, we soon detect a ball flying out
      if (pca.powerCellFired())
       {
        System.out.println("Shot at " + pca.getShooterRPM() + " RPMs");
        state = State.SUCCESS;
       }
        // In reality, we might not, so stop after a few seconds
      else if (timeout_timer.hasElapsed(5.0))
        state = State.TIMEOUT;
    }
  }

  @Override
  public boolean isFinished()
  {
    return state == State.SUCCESS  ||  state == State.TIMEOUT;
  }

  @Override
  public void end(final boolean interrupted)
  {
    System.out.println("EJECT: " + state);
    // Turn ejector off
    // (but it keeps running for a while in case we want to shoot again, soon)
    pca.feedEjector(false);
    pca.eject(false);
  }
}
