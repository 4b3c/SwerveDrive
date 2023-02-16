package frc.robot.Auto;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class DriveRoute {

    public static int index = 0;
    public static double[] target = {0, 0};

    //drive the ZigZag path using a PID to go to each point along the curve
    public static void drivePID() {
        if (DriveTo.goToCoordsPID(SimpleZigZag.path[index][0], SimpleZigZag.path[index][1], 100)) {
            index = index + 1;
            if (index == SimpleZigZag.path.length) {
                index = 0;
            }
        }
        SmartDashboard.putNumber("target x", SimpleZigZag.path[index][0]);
        SmartDashboard.putNumber("target y", SimpleZigZag.path[index][1]);
    }

    //drive the ZigZag path without a PID, rather with just a set speed
    public static void driveNoPID() {
        if (DriveTo.goToCoords(SimpleZigZag.path[index][0], SimpleZigZag.path[index][1], 0.2, 100)) {
            index = index + 1;
            if (index == SimpleZigZag.path.length) {
                index = 0;
            }
        }
        SmartDashboard.putNumber("target x", SimpleZigZag.path[index][0]);
        SmartDashboard.putNumber("target y", SimpleZigZag.path[index][1]);
    }

    //drive the ZigZag path using a PID but with a big cutoff so it starts moving to the next point before it slows down
    public static void drivePIDCutOff() {
        if (DriveTo.goToCoordsPID(SimpleZigZag.path[index][0], SimpleZigZag.path[index][1], 500)) {
            index = index + 1;
            if (index == SimpleZigZag.path.length) {
                index = 0;
            }
        }
        SmartDashboard.putNumber("target x", SimpleZigZag.path[index][0]);
        SmartDashboard.putNumber("target y", SimpleZigZag.path[index][1]);
    }

    //drive the ZigZag path using a PID for the whole route
    public static void driveIndexPID() {
        if (DriveTo.goToCoordsPID2(SimpleZigZag.path[index][0], SimpleZigZag.path[index][1], index, SimpleZigZag.path.length, 500)) {
            index = index + 1;
            if (index == SimpleZigZag.path.length) {
                index = 0;
            }
        }
        SmartDashboard.putNumber("target x", SimpleZigZag.path[index][0]);
        SmartDashboard.putNumber("target y", SimpleZigZag.path[index][1]);
    }

}
