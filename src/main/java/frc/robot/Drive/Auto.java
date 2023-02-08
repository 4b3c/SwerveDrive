package frc.robot.Drive;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Map;

public class Auto {

    public static PIDController distAutoPID = new PIDController(0.01, 0.0001, 0.0);;
    public static double speedAuto;
    public static double angleAuto;
    public static double distAuto;
    
    public static void returnToOrigin()
    {
        angleAuto = -Wheel.toDegrees(Math.atan2(Map.swerve.yPos, -Map.swerve.xPos));
        distAuto = Math.sqrt(Map.swerve.yPos * Map.swerve.yPos + Map.swerve.xPos * Map.swerve.xPos);
        speedAuto = Math.abs(distAutoPID.calculate(distAuto / 40));
        if (speedAuto > 0.3) {
            speedAuto = 0.3;
        }

        Map.swerve.swerveDrive(angleAuto, speedAuto, 0);
        Map.swerve.odometry();

        SmartDashboard.putNumber("angle auto", angleAuto);
        SmartDashboard.putNumber("speed auto", speedAuto);
        SmartDashboard.putBoolean("Auto", true);
    }
}
