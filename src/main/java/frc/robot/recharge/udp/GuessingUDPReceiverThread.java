/*----------------------------------------------------------------------------*/
/* Copyright (c) 2020 FIRST Team 2393. All Rights Reserved.                   */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/
package frc.robot.recharge.udp;

/** Thread that keeps reading from UDPClient, using Guesstimator */
public class GuessingUDPReceiverThread
{
  private final UDPClient client;
  private final Thread thread;
  // SYNC on access
  private final Guesstimator guesstimator = new Guesstimator();

  public GuessingUDPReceiverThread(final int port) throws Exception
  {
    client = new UDPClient(port);
    thread = new Thread(this::receive);
    thread.setDaemon(true);
    thread.start();
  }

  private void receive()
  {
    long last_ms = 0;
    try
    {
      while (true)
      {
        // Wait for update from camera..
        final CameraData update = client.read();

        // Camera may send the same UDP packet multiple times,
        // so ignore packet that arrives just a few ms later
        if ( update.millisec - last_ms < 5)
          continue;
          
        last_ms = update.millisec;
        // .. and add to guesstimator
        synchronized(guesstimator)
        {
          guesstimator.addData(update);
        }
      }
    }
    catch (Exception ex)
    {
      ex.printStackTrace();
    }
    System.err.println("GuessingUDPReceiverThread quits");
  }

  /** @return Guesstimated camera data */
  public CameraData get()
  {
    synchronized (guesstimator)
    {
      return guesstimator.guesstimate();
    }
  }
}
