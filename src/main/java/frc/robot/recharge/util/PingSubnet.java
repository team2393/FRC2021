/*----------------------------------------------------------------------------*/
/* Copyright (c) 2020 FIRST Team 2393. All Rights Reserved.                   */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.recharge.util;

import java.net.InetAddress;
import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.CountDownLatch;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;

/** Ping all IP addresses in subnet to look for devices on the network */
public class PingSubnet
{
  // We start a couple of threads to perform the check in parallel.
  // Each thread, successful or not, will count this latch down
  // so we know when all are done.
  private final CountDownLatch done = new CountDownLatch(254);

  // List of IP addresses that replied to our 'ping'.
  // Not per se thread-save, so need to synchronize!
  private final List<String> responses = new ArrayList<>();


  public PingSubnet()
  {
    // Submit lookups for IP 10.0.0.1 to 10.0.0.254,
    // handling 20 in parallel
    final ExecutorService executors = Executors.newFixedThreadPool(20);
    for (int i = 1; i <= 254; ++i)
    {
      final String ip = "10.23.93." + i;
      executors.execute(() -> lookup(ip));
    }

    // Wait until they're all done
    try
    {
      done.await();
    }
    catch (InterruptedException ex)
    {
      // Ignore
    }
    executors.shutdown();

    // Show (sorted) responses
    responses.sort(String::compareTo);
    System.out.println("Network devices");
    System.out.println("---------------");
    for (String device : responses)
      System.out.println(device);
    System.out.println("---------------");
  }

  private void lookup(final String ip)
  {
    try
    {
      // Try for 1 second to 'ping' or connect to 'echo' port
      if (InetAddress.getByName(ip).isReachable(1000))
        synchronized (responses)
        { // Add to the sync'ed list
          responses.add(ip);
        }
    }
    catch (Exception ex)
    {
      ex.printStackTrace();
    }
    done.countDown();
  }
}
