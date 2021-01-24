/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST Team 2393. All Rights Reserved.                   */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.demo.commands;

import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj2.command.CommandBase;

/** Command that blinks (or beeps)
 * 
 *  Periodically turns an output on/off
 */
public class Blink extends CommandBase
{
  private final DigitalOutput output;
  private final long period_ms;
  private long start_ms = 0;

  /** @param output Output to use
   *  @param period_ms On/off period in milliseconds
   */
  public Blink(final DigitalOutput output, final long period_ms)
  {
    this.output = output;
    this.period_ms = period_ms;
  }

  @Override
  public void initialize()
  {
    // Remember when command started so that
    // we always blink with a full 'on' period
    // when started, instead of initially being on/off
    // depending on the currentTimeMillis
    start_ms = System.currentTimeMillis();
  }

  @Override
  public void execute()
  {
    // Determine if output should be on or off:
    // Check how much time has passed since start_ms,
    // convert into periods,
    // turn 'on' for all even periods, including period 0.
    final boolean on_off = ((System.currentTimeMillis() - start_ms) / period_ms) % 2 == 0;
    output.set(on_off);
  }

  @Override
  public void end(boolean interrupted)
  {
    // When command ends, ensure that output is off
    output.set(false);
  }
}
