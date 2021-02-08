/*----------------------------------------------------------------------------*/
/* Copyright (c) 2020 FIRST Team 2393. All Rights Reserved.                   */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.recharge.shooter;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.recharge.shooter.PowerCellAccelerator.State;

/** Eject one power cell.
 *  Since ejector keeps running a little longer,
 *  rescheduling this command right away keeps
 *  the cells flowing.
 */
public class Eject extends CommandBase
{
  private final PowerCellAccelerator pca;
  private final Timer timeout_timer = new Timer();
  private boolean timeout = false;

  public Eject(final PowerCellAccelerator pca)
  {
    this.pca = pca;
    addRequirements(pca);
  }

  @Override
  public void initialize()
  {
    System.out.println("Started Eject");
    pca.setState(PowerCellAccelerator.State.SPIN_UP);
    timeout_timer.reset();
    timeout_timer.start();
  }

  @Override
  public void execute()
  {
     if (timeout_timer.hasElapsed(5.0))
     {
        timeout = true;
        System.out.println("** Eject timed out **");
        pca.setState(State.LOAD);
     }
  }

  @Override
  public boolean isFinished()
  {
     return timeout ||
            ! (pca.getState() == State.SPIN_UP  ||  pca.getState() == State.EJECT);
  }

  @Override
  public void end(boolean interrupted) 
  {
    super.end(interrupted);
    System.out.println("Ended Eject After " + timeout_timer.get() + " Seconds");
  }
}
