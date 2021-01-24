/*----------------------------------------------------------------------------*/
/* Copyright (c) 2020 FIRST Team 2393. All Rights Reserved.                   */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/
package frc.robot.recharge.udp;

import java.net.InetSocketAddress;
import java.net.InterfaceAddress;
import java.net.NetworkInterface;
import java.net.StandardProtocolFamily;
import java.nio.ByteBuffer;
import java.nio.channels.DatagramChannel;
import java.util.ArrayList;
import java.util.Collections;
import java.util.List;
import java.util.concurrent.TimeUnit;

/** Send a number via UDP */
public class UDPServer
{
  private final DatagramChannel udp;
  private final ByteBuffer buffer = ByteBuffer.allocate(Integer.BYTES);
  private final List<InetSocketAddress> broadcasts = new ArrayList<>();

  public UDPServer(final int port) throws Exception
  {
    // Create a 'socket' that can use broadcasts
    udp = DatagramChannel.open(StandardProtocolFamily.INET);
    udp.configureBlocking(true);
    udp.socket().setBroadcast(true);
    udp.socket().setReuseAddress(true);

    // Find all network interfaces that support broadcast
    for (NetworkInterface iface : Collections.list(NetworkInterface.getNetworkInterfaces()))
      if (iface.isUp())
        for (InterfaceAddress addr : iface.getInterfaceAddresses())
          if (addr.getBroadcast() != null)
          broadcasts.add(new InetSocketAddress(addr.getBroadcast(), port));

    System.out.println("UDP Server broadcasting to " + broadcasts);
  }

  public void send(final int number) throws Exception
  {
    // Place number in byte buffer
    buffer.clear();
    buffer.putInt(number);
    
    // Send as broadcast
    for (InetSocketAddress addr : broadcasts)
    {
      buffer.flip();
      udp.send(buffer, addr);
    }
  }

  public static void main(String[] args) throws Exception
  {
    // Example server that sends numbers at about 30 Hz
    final UDPServer server = new UDPServer(5801);
    for (int number = 1;  true;  ++number)
    {
      server.send(number);
      TimeUnit.MILLISECONDS.sleep(1000/30);
    }  
  }
}
