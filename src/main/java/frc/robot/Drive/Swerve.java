package frc.robot.Drive;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Map;

public class Swerve {

    private int[][] ports = {{1, 5, 9}, {2, 6, 10}, {3, 7, 11}, {4, 8, 12}};
    private double[][] sumXY = {{0, 0}, {0, 0}, {0, 0}, {0, 0}};
    private double sumX;
    private double sumY;

    private Wheel wheelFR;
    private Wheel wheelFL;
    private Wheel wheelBL;
    private Wheel wheelBR;

    private double cycleTime;
    public double xPos;
    public double yPos;
    public double[] coords = {0, 0};

    //offset, module numbers, id's for rotate and for drive, rotation, drive, and angle
    public Swerve()
    {
        this.wheelFR = new Wheel("FR", 148.7, ports[0]);
        this.wheelFL = new Wheel("FL", 115.1, ports[1]);
        this.wheelBL = new Wheel("BL", 34.4, ports[2]);
        this.wheelBR = new Wheel("BR", 107.7, ports[3]);
    }

    public void swerveDrive(double angle, double speed, double twist)
    {
        SmartDashboard.putNumber("angle", angle);
        if (speed > Map.deadband || Math.abs(twist) > Map.deadband) {
            if (speed > Map.deadband) {
                speed = speed - Map.deadband;
            } else {
                speed = 0;
            }
            if (Math.abs(twist) > Map.deadband) {
                if (twist < 0) {
                    twist = twist - Map.deadband;
                } else {
                    twist = twist + Map.deadband;
                }
            } else {
                twist = 0;
            }
            
            this.wheelFR.drive(angle, speed, twist);
            this.wheelFL.drive(angle, speed, twist);
            this.wheelBL.drive(angle, speed, twist);
            this.wheelBR.drive(angle, speed, twist);
        } else {
            this.wheelFR.stop();
            this.wheelFL.stop();
            this.wheelBL.stop();
            this.wheelBR.stop();
        }
    }

    public void odometry(double robotAngle)
    {
        sumXY[0] = this.wheelFR.changeInXY;
        sumXY[1] = this.wheelFL.changeInXY;
        sumXY[2] = this.wheelBL.changeInXY;
        sumXY[3] = this.wheelBR.changeInXY;

        sumX = (sumXY[0][0] + sumXY[1][0] + sumXY[2][0] + sumXY[3][0]) / 4;
        sumY = (sumXY[0][1] + sumXY[1][1] + sumXY[2][1] + sumXY[3][1]) / 4;

        cycleTime = Timer.getFPGATimestamp() - Map.elapsedTime;
        Map.elapsedTime += cycleTime;
        this.xPos += sumX * cycleTime;
        this.yPos += sumY * cycleTime;

        this.coords[0] = (Math.sin(Wheel.toRadians(robotAngle + 90)) + Math.cos(Wheel.toRadians(robotAngle + 90)) - 1) * 500;
        this.coords[1] = (Math.sin(Wheel.toRadians(robotAngle)) + Math.cos(Wheel.toRadians(robotAngle)) - 1) * 525;

        SmartDashboard.putNumber("x pos", this.xPos + coords[0]);
        SmartDashboard.putNumber("y pos", this.yPos - coords[1]);

        SmartDashboard.putNumber("x change", coords[0]);
        SmartDashboard.putNumber("y change", coords[1]);
    }
}
