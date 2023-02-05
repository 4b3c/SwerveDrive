package frc.robot;

import com.ctre.phoenix.sensors.Pigeon2;
import edu.wpi.first.wpilibj.Joystick;
import frc.robot.Drive.Swerve;

public class Map {
    
    public static Joystick driver = new Joystick(0);
    public static Swerve swerve = new Swerve();

    public static Pigeon2 gyro = new Pigeon2(0);
    public static double initialAngle;
    
    public static double deadband = 0.1;

    public static double elapsedTime;
}
