/*----------------------------------------------------------------------------*/
/* Copyright (c) 2020 FIRST Team 2393. All Rights Reserved.                   */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/
package frc.robot.recharge.udp;

import java.net.InetSocketAddress;
import java.net.StandardProtocolFamily;
import java.nio.ByteBuffer;
import java.nio.channels.DatagramChannel;
import java.time.LocalDateTime;

/** Receive a number via UDP */
public class UDPClient
{
  private final DatagramChannel udp;
  private final ByteBuffer buffer = ByteBuffer.allocate(Integer.BYTES*2);

  public UDPClient(final int port) throws Exception
  {
    udp = DatagramChannel.open(StandardProtocolFamily.INET);
    udp.configureBlocking(true);
    udp.socket().setBroadcast(true);
    udp.socket().setReuseAddress(true);
    udp.bind(new InetSocketAddress(port));
    System.out.println("UDP Client listening on " + udp.getLocalAddress());
  }

  public CameraData read() throws Exception
  {
    // Read <whatever> into buffer
    // (blocks until we receive something)
    buffer.clear();
    udp.receive(buffer);
    
    // Assume that the buffer now contains a number
    buffer.flip();
    int direction = buffer.getInt();
    int distance = buffer.getInt();
    return new CameraData(direction, distance);
  }

  public static void main(String[] args) throws Exception
  {
    final UDPClient client = new UDPClient(5801);
    while (true)
    {
      final CameraData data = client.read();
      System.out.println(LocalDateTime.now() + " " + data);
    }  
  }
}
