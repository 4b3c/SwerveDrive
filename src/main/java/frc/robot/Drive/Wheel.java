// CTRE LIBRARY: https://maven.ctr-electronics.com/release/com/ctre/phoenixpro/PhoenixProAnd5-frc2023-latest.json

package frc.robot.Drive;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.CANCoder;

import edu.wpi.first.math.controller.PIDController;

public class Wheel {
    
    private String id;
    private double offset;
    private TalonFX driveMotor;
    private TalonFX rotateMotor;
    private CANCoder angleSensor;
    private int rotateAngle;

    private double[] strafeVector = {0, 0};
    private double[] rotateVector = {0, 0};
    private double[] driveVector = {0, 0};
    private double[] shortcut = {0, 0};

    private PIDController anglePID;
    private PIDController speedPID;

    private double currentAngle;
    private double currentSpeed;
    public double[] changeInXY = {0, 0};

    //wheel object initiallizer takes in an id, the wheel's offset and a int array of the drive, rotate motors and CANCoder ports
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

        this.anglePID = new PIDController(0.0, 0.0, 0.0);
        this.speedPID = new PIDController(0.0, 0.0, 0.0);

        //set the rotation angle based on which wheel it is
        switch (this.id) {
            case "FR": this.rotateAngle = 135;
            case "FL": this.rotateAngle = 225;
            case "BL": this.rotateAngle = 315;
            case "BR": this.rotateAngle = 45;
        }
    }

    public void drive(double angle, double speed, double twist)
    {
        //get our strafe and rotate vectors from the inputs
        strafeVector[0] = angle;
        strafeVector[1] = speed;
        rotateVector[0] = this.rotateAngle;
        rotateVector[1] = twist;

        //get the current and angle and speed of the wheel
        currentAngle = this.angleSensor.getAbsolutePosition();
        currentSpeed = this.driveMotor.getSelectedSensorVelocity();

        //combine the strafe and rotate vectors into a drive vector and find the shortest path
        driveVector = addVectors(strafeVector, rotateVector);
        shortcut = elOptimal(currentAngle, driveVector[1]);

        //set the rotate and drive motors to the calculated velocities
        this.driveMotor.set(ControlMode.Velocity, this.speedPID.calculate(currentSpeed, driveVector[0] * shortcut[1]));
        this.rotateMotor.set(ControlMode.Velocity, this.anglePID.calculate(shortcut[0]));

        //calculate the x and y speeds of the wheel
        odometry(currentAngle, currentSpeed);
    }

    //make sure the motors stop moving
    public void stop() {
        this.driveMotor.set(ControlMode.Velocity, 0);
        this.rotateMotor.set(ControlMode.Velocity, 0);
    }

    //returns the x and y speeds of the wheel in ticks/100ms
    public void odometry(double currentAngle, double currentSpeed)
    {
        //split the motor speed into the x and y velocities of the wheel using trig
        this.changeInXY[0] = Math.sin(toRadians(currentAngle)) * currentSpeed;
        this.changeInXY[1] = Math.cos(toRadians(currentAngle)) * currentSpeed;
 
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

    //Adds two vectors {magnitude, angle}
    public static double[] addVectors(double[] arr1, double[] arr2) {
        double magnitudeOne = arr1[0];
        double angleOne = arr1[1];

        double magnitudeTwo = arr2[0];
        double angleTwo = arr2[1];

        //split the vectors into x and y and add the two Xs and two Ys and add them together
        double newX = (magnitudeOne * Math.cos(toRadians(angleOne))) + (magnitudeTwo * Math.cos(toRadians(angleTwo)));
        double newY = (magnitudeOne * Math.sin(toRadians(angleOne))) + (magnitudeTwo * Math.sin(toRadians(angleTwo)));

        //turn the total Xs and Ys into a new vector
        double newAngle = toDegrees(Math.atan2(newY, newX));
        double newMag = Math.sqrt((newX*newX) + (newY * newY));

        double[] resultingVector = {newMag, newAngle};

        return resultingVector;
    }

    //converts degrees to radians
    public static double toRadians(double angle) {
        return (angle * Math.PI) / 180;
    }

    //converts radians to degrees
    public static double toDegrees(double angle) {
        return (angle * 180) / Math.PI;
    }

}
