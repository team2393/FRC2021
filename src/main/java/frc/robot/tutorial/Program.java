package frc.robot.tutorial;

/** Compile this with
 *
 *    javac Program.java
 *
 *  and then check the result via
 *
 *   javap -c Program.class
 *
 *  Can you find the number 65 in the byte code?
 *  Where's the 'A'?
 */
class Program
{
    public static void main(String argv[])
    {
        int number = 65;
        char character = 'A';
        System.out.println(number + character);
    }
}
