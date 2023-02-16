package frc.robot.Auto;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Map;
import frc.robot.Drive.Wheel;

public class DriveTo {

    //create a pid controller for controlling the driving speed going to coordinates
    public static PIDController distAutoPID = new PIDController(0.02, 0.0001, 0.0);;
    public static double speedAuto;
    public static double angleAuto;
    public static double distAuto;
    public static double distanceY;
    public static double distanceX;
    
    //function to drive the swerve to an x & y coordinates fieldcentric using a pid
    public static boolean goToCoordsPID(double x, double y, double cutoff)
    {
        //get the distance from the target to the current position
        distanceX = x - (Map.swerve.xPos + Map.swerve.coords[0]);
        distanceY = y - (Map.swerve.yPos - Map.swerve.coords[1]);
        //using the x & y distance, get the angle to the target, and the distance using pythagoras theorem
        angleAuto = Wheel.toDegrees(Math.atan2(distanceY, distanceX));
        distAuto = Math.sqrt(distanceY * distanceY + distanceX * distanceX);
        //then calculate the speed using the pid
        speedAuto = Math.abs(distAutoPID.calculate(distAuto));
        if (speedAuto > 0.3) {
            speedAuto = 0.3;
        }

        //drive the swerve at the calculated speed and angle, then calculate odometry
        Map.swerve.swerveDrive(angleAuto + Map.initialAngle - Map.gyro.getYaw(), speedAuto, 0);
        Map.swerve.odometry(Map.initialAngle - Map.gyro.getYaw());

        SmartDashboard.putNumber("angle auto", angleAuto);
        SmartDashboard.putNumber("speed auto", speedAuto);
        SmartDashboard.putNumber("dist auto", distAuto);

        //return true when the distance is less than 100 ticks
        if (distAuto < cutoff) {
            return true;
        }
        return false;

    }

    //function to drive the swerve to an x & y coordinates fieldcentric without a pid
    public static boolean goToCoords(double x, double y, double speed, double cutoff)
    {
        //get the distance from the target to the current position
        distanceX = x - (Map.swerve.xPos + Map.swerve.coords[0]);
        distanceY = y - (Map.swerve.yPos - Map.swerve.coords[1]);
        //using the x & y distance, get the angle to the target, and the distance using pythagoras theorem
        angleAuto = Wheel.toDegrees(Math.atan2(distanceY, distanceX));
        distAuto = Math.sqrt(distanceY * distanceY + distanceX * distanceX) / 400;
        if (distAuto > 0.3) {
            distAuto = 0.3;
        }

        //drive the swerve at the calculated speed proportional to distance, and angle, then calculate odometry
        Map.swerve.swerveDrive(angleAuto + Map.initialAngle - Map.gyro.getYaw(), distAuto, 0);
        Map.swerve.odometry(Map.initialAngle - Map.gyro.getYaw());

        SmartDashboard.putNumber("angle auto", angleAuto);
        SmartDashboard.putNumber("dist auto", distAuto);

        //return true when the distance is less than 100 ticks
        if (distAuto < cutoff) {
            return true;
        }
        return false;

    }

    //function to drive the swerve to an x & y coordinates fieldcentric using a pid
    public static boolean goToCoordsPID2(double x, double y, double index, double lengthOfPoints, double cutoff)
    {
        //get the distance from the target to the current position
        distanceX = x - (Map.swerve.xPos + Map.swerve.coords[0]);
        distanceY = y - (Map.swerve.yPos - Map.swerve.coords[1]);
        //using the x & y distance, get the angle to the target, and the distance using pythagoras theorem
        angleAuto = Wheel.toDegrees(Math.atan2(distanceY, distanceX));
        distAuto = Math.sqrt(distanceY * distanceY + distanceX * distanceX);
        //then calculate the speed using the pid
        speedAuto = Math.abs(distAutoPID.calculate(lengthOfPoints - index));
        if (speedAuto > 0.3) {
            speedAuto = 0.3;
        }

        //drive the swerve at the calculated speed and angle, then calculate odometry
        Map.swerve.swerveDrive(angleAuto + Map.initialAngle - Map.gyro.getYaw(), speedAuto, 0);
        Map.swerve.odometry(Map.initialAngle - Map.gyro.getYaw());

        SmartDashboard.putNumber("angle auto", angleAuto);
        SmartDashboard.putNumber("speed auto", speedAuto);
        SmartDashboard.putNumber("dist auto", distAuto);

        //return true when the distance is less than 100 ticks
        if (distAuto < cutoff) {
            return true;
        }
        return false;

    }

    
}
