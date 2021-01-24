/*----------------------------------------------------------------------------*/
/* Copyright (c) 2020 FIRST Team 2393. All Rights Reserved.                   */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/
package frc.robot.recharge.udp;

import java.util.concurrent.atomic.AtomicReference;

/** Thread that keeps reading from UDPClient, remembering the latest value */
public class UDPReceiverThread
{
  /** The latest number, 'atomic' because accessed by
   *  a) Receiving thread
   *  b) Thread that checks what we received
   */
  private final AtomicReference<CameraData> data = new AtomicReference<>();
  private final UDPClient client;
  private final Thread thread;

  public UDPReceiverThread(final int port) throws Exception
  {
    client = new UDPClient(port);
    thread = new Thread(this::receive);
    thread.setDaemon(true);
    thread.start();
  }

  private void receive()
  {
    try
    {
      while (true)
      {
        data.set(client.read());
      }
    }
    catch (Exception ex)
    {
      ex.printStackTrace();
    }
    System.err.println("Receive thread quits");
  }

  /** @return Latest value or 'null' if stale */
  public CameraData get()
  {
    // Get the latest value and then mark it as stale.
    // When we're called next time, either
    // a) We did receive a new value -> Good
    // b) No new value -> Caller can see that it's STALE
    return data.getAndSet(null);
  }
}
