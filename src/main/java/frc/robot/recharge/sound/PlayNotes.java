/*----------------------------------------------------------------------------*/
/* Copyright (c) 2020 FIRST Team 2393. All Rights Reserved.                   */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.recharge.sound;

import java.util.List;

import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;

/** Command that plays a sequence of notes on Talons */
public class PlayNotes extends CommandBase
{
  // https://en.wikipedia.org/wiki/C_(musical_note)
  public static final int OFF = 0;
  public static final int C4 = 261;
  public static final int A4 = 440;
  public static final int C5 = 2*261;
  

  private final List<TalonFX> instruments;
  private final int[] notes;
  private final Timer timer = new Timer();
  /** Time in seconds between each note */
  private final double period = 0.1;
  private final int repeat;
  private int times;

  /** Play notes on instrumments
   * 
   *  @param instruments Instruments (all will play the same notes)
   *  @param repeat How often to repeat the sequence of notes
   *  @param notes Notes to play, i.e. frequencies in Hz
   */
  public PlayNotes(final List<TalonFX> instruments,
                   final int repeat,
                   final int[] notes)
  {
    this.instruments = instruments;
    this.repeat = repeat;
    this.notes = notes;
  }

  @Override
  public void initialize()
  {
    timer.start();
    times = repeat;
  }

  @Override
  public void execute()
  {
    int note = (int) (timer.get() / period);
    if (note >= notes.length)
    { // End of sequence
      // Start over on first note (and maybe end)
      note = 0;
      timer.reset();
      --times;
    }
    for (TalonFX talon : instruments)
      talon.set(TalonFXControlMode.MusicTone, notes[note]);
  }

  @Override
  public boolean isFinished()
  {
    return times <= 0;
  }

  @Override
  public void end(final boolean interrupted)
  {
    for (TalonFX talon : instruments)
      talon.set(TalonFXControlMode.MusicTone, OFF);
  }
}
