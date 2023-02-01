package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import frc.robot.Drive.Swerve;

public class Map {
    
    public static Joystick driver = new Joystick(0);
    public static Swerve swerve = new Swerve();
    
    public static double deadband = 0.1;
}
