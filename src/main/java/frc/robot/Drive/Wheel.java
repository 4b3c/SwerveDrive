// CTRE LIBRARY: https://maven.ctr-electronics.com/release/com/ctre/phoenixpro/PhoenixProAnd5-frc2023-latest.json

package frc.robot.Drive;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.CANCoder;

public class Wheel {
    
    private String id;
    private double offset;
    private TalonFX driveMotor;
    private TalonFX rotateMotor;
    private CANCoder angleSensor;
    private int rotateAngle;

    public Wheel(String id, double offset, int[] ports)
    {
        this.id = id;
        this.offset = offset;

        //create the hardware objects of drive, rotate motors and the CANCoder sensor
        this.driveMotor = new TalonFX(ports[0]);
        this.rotateMotor = new TalonFX(ports[1]);
        this.angleSensor = new CANCoder(ports[2]);

        //set defaults for the hardware objects
        this.driveMotor.setNeutralMode(NeutralMode.Brake);
        this.rotateMotor.setNeutralMode(NeutralMode.Brake);
        this.angleSensor.setPosition(this.offset);  // don't know if this works lmao ---------------------------------------------- gottem

        //set the rotation angle based on which wheel it is
        switch (this.id) {
            case "FR": this.rotateAngle = 45;
            case "FL": this.rotateAngle = 135;
            case "BL": this.rotateAngle = 225;
            case "BR": this.rotateAngle = 315;
        }
    }

    public void drive(double angle, double speed, double twist)
    {
        //do stuff
    }

    public double[] odometry()
    {
        //split the motor speed into the x and y velocities of the wheel using trig and return that
        double x = Math.sin(toRadians(this.angleSensor.getAbsolutePosition())) * this.driveMotor.getSelectedSensorVelocity();
        double y = Math.cos(toRadians(this.angleSensor.getAbsolutePosition())) * this.driveMotor.getSelectedSensorVelocity();
        double[] result = {x, y};
        return result;
 
    }

    // Returns a double array {distance to closest target, reverse variable}
    public static double[] elOptimal(double currentAngle, double targetAngle)
    {
        double[] diffAndReverse = {0, 1};
        double diff = currentAngle - targetAngle;

        // If the target and current are within 90 degrees, just return the difference
        if (diff < 90 && diff >= -90) {
            diffAndReverse[0] = diff;
            return diffAndReverse;
        
        // If it's more than 90 but less than 270 degrees, we return the difference to the opposite angle
        } else if (diff <= 270 && diff >= -270) {
            if (diff > 0) {
                diffAndReverse[0] = diff - 180;
                diffAndReverse[1] = -1;
                return diffAndReverse;

            } else {
                diffAndReverse[0] = diff + 180;
                diffAndReverse[1] = -1;
                return diffAndReverse;
            }
        
        // If it's more than 270 degrees, 
        } else {
            if (diff > 0) {
                diffAndReverse[0] = diff - 360;
                return diffAndReverse;

            } else {
                diffAndReverse[0] = diff + 360;
                return diffAndReverse;
            }
        }
    }

    public static double toRadians(double angle) {
        return (angle * Math.PI) / 180;
    }

    public static double toDegrees(double angle) {
        return (angle * 180) / Math.PI;
    }

}
