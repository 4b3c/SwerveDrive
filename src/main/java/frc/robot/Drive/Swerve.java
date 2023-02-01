package frc.robot.Drive;

import edu.wpi.first.wpilibj.Timer;
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
    private double elapsedTime;
    public double xPos;
    public double yPos;

    //offset, module numbers, id's for rotate and for drive, rotation, drive, and angle
    public Swerve()
    {
        this.wheelFR = new Wheel("FR", 238.7 + 90, ports[3]);
        // this.wheelFL = new Wheel("FL", 200.1 + 90, ports[2]);
        // this.wheelBL = new Wheel("BL", 6.4 + 90, ports[1]);
        // this.wheelBR = new Wheel("BR", 116.7 + 90, ports[0]);
    }

    public void swerveDrive(double angle, double speed, double twist)
    {
        if (speed > Map.deadband || twist > Map.deadband) {
            this.wheelFR.drive(angle, speed, twist);
            // this.wheelFL.drive(angle, speed, twist);
            // this.wheelBL.drive(angle, speed, twist);
            // this.wheelBR.drive(angle, speed, twist);
        } else {
            this.wheelFR.stop();
            // this.wheelFL.stop();
            // this.wheelBL.stop();
            // this.wheelBR.stop();
        }
    }

    public void odometry()
    {
        sumXY[0] = this.wheelFR.changeInXY;
        sumXY[1] = this.wheelFL.changeInXY;
        sumXY[2] = this.wheelBL.changeInXY;
        sumXY[3] = this.wheelBR.changeInXY;

        sumX = (sumXY[0][0] + sumXY[1][0] + sumXY[2][0] + sumXY[3][0]) / 4;
        sumY = (sumXY[0][1] + sumXY[1][1] + sumXY[2][1] + sumXY[3][1]) / 4;

        cycleTime = Timer.getFPGATimestamp() - elapsedTime;
        elapsedTime += cycleTime;
        this.xPos += sumX * cycleTime;
        this.yPos += sumY * cycleTime;
    }
}
