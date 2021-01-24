package frc.robot.demo.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.PrintCommand;

public class PerpetualDemo
{
  public static void main(final String[] args)
  {
    // This looks like it should keep printing 'Hello':
    final Command demo1 = new PrintCommand("Hello").perpetually();
    // This 'perpetual' command never ends, but only prints 'Hello'
    // once because the print command only prints during initialize
    demo1.initialize();
    demo1.execute();
    demo1.execute();
    demo1.execute();
  
    System.out.println("----");

    final Command demo2 = new PrintCommand("Hello").andThen(new PrintCommand("Bye!")).perpetually();
    demo2.initialize();
    demo2.execute();
    demo2.execute();
    demo2.execute();

    /*
    java.lang.IndexOutOfBoundsException: Index 2 out of bounds for length 2
        at java.base/jdk.internal.util.Preconditions.outOfBounds(Preconditions.java:64)
        at java.base/jdk.internal.util.Preconditions.outOfBoundsCheckIndex(Preconditions.java:70)
        at java.base/jdk.internal.util.Preconditions.checkIndex(Preconditions.java:248)
        at java.base/java.util.Objects.checkIndex(Objects.java:372)
        at java.base/java.util.ArrayList.get(ArrayList.java:458)
        at edu.wpi.first.wpilibj2.command.SequentialCommandGroup.execute(SequentialCommandGroup.java:66)
        at edu.wpi.first.wpilibj2.command.PerpetualCommand.execute(PerpetualCommand.java:44)
        */
  }
}