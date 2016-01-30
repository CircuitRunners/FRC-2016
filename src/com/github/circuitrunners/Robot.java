
package com.github.circuitrunners;

import edu.wpi.first.wpilibj.AnalogGyro;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.RobotDrive;

import java.lang.reflect.Constructor;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;

public class Robot extends IterativeRobot {

    RobotDrive drive;

    VictorSP motor;

    Joystick joystick;

    AnalogGyro gyro;

    @Override
    public void robotInit() {

        drive = new RobotDrive(5,4,1,0);
        motor = new VictorSP(3);

        joystick = new Joystick(0);

        gyro = new AnalogGyro(0);

        drive.setExpiration(0.5);

    }

    @Override
    public void autonomousInit() {
        ExecutorService parallelExecutor = Executors.newCachedThreadPool();
        ExecutorService sequentialExecutor = Executors.newSingleThreadExecutor();
        sequentialExecutor.execute(new AutonomousDriveThread(1, 0, 1000));
        sequentialExecutor.execute(new AutonomousDriveThread(-1, 0, 1000));
    }

    private class AutonomousDriveThread implements Runnable {
        double moveVal;
        double rotateVal;
        int waitTime;

        public AutonomousDriveThread(double moveVal, double rotateVal) {
            this.moveVal = moveVal;
            this.rotateVal = rotateVal;
        }

        public AutonomousDriveThread(double moveVal, double rotateVal, int waitTime) {
            this.moveVal = moveVal;
            this.rotateVal = rotateVal;
            this.waitTime = waitTime;
        }

        @Override
        public void run() {
            drive(moveVal, rotateVal);
            if (waitTime > 0) {
                try {
                    Thread.sleep(waitTime);
                } catch (InterruptedException e) {
                    e.printStackTrace();
                }
            }
        }
    }

    @Override
    public void autonomousPeriodic() {

    }

    @Override
    public void teleopInit() {
        SmartDashboard.putNumber("derp", 0);

        SmartDashboard.putNumber("pid_kP", pidController.getP());
        SmartDashboard.putNumber("pid_kI", pidController.getI());
        SmartDashboard.putNumber("pid_kD", pidController.getD());
        pidController.enable();
    }
    boolean triggerPressed;
    @Override
    public void teleopPeriodic() {
        drive(joystick.getY(), joystick.getX());
    }

    public void drive(double moveVal, double rotateVal) {
//        double gyroVal = gyro.getAngle();
//
//        if (gyroVal != Math.asin(rotateVal)) {
//            if (gyroVal < 357 && gyroVal > 183) {
//                rotateVal -= Math.sin(gyroVal);
//            } else if (gyroVal > 3 && gyroVal < 177) {
//                rotateVal += Math.sin(gyroVal);
//            }
//        }
        double moveVal = -joystick.getY(); // wtf are directions
        double twistVal = -joystick.getTwist(); //jesus cant save you now
        double throttleVal = -joystick.getThrottle(); //why is everything negative?

        double rotateVal = scalePower(twistVal, 0.1, 0.7, 2); //could make magic numbers into constants but who cares
        double throttledMove = throttleMath(throttleVal) * moveVal; //0% chance we need this elsewhere but who cares

        drive.arcadeDrive(throttledMove, rotateVal);

        SmartDashboard.putNumber("moveVal", moveVal);
        SmartDashboard.putNumber("rotateVal", rotateVal);
        SmartDashboard.putNumber("gyroVal", gyroVal);

        motor.set(SmartDashboard.getNumber("derp",0));
    }

    private double scaleLinear(double input, double min, double scale) {
        double output = 0;
        if(Math.abs(input) < min) {
            output = 0;
        }
        else {
            output = scale * input;
        }
        return output;
    }

    private double scaleDoubleFlat(double input, double min, double scale, double power){
        double output = 0;
        if(Math.abs(input) < min) {
            output = 0;
        }
        else if(Math.abs(input) > scale){
            output = Math.signum(input) * scale;
        }
        else{
            output = scale * Math.signum(input) * Math.abs(Math.pow(input/scale,power));
        }
        return output;
    }

    private double scalePower(double input, double min, double scale, double power){
        double output = 0;
        if(Math.abs(input) < min) {
            output = 0;
        }
        else {
            output = scale * Math.signum(input) * Math.abs(Math.pow(input,power));
        }
        return output;
    }

    private double throttleMath(double input){
        double output = (input+1)/2;
        return output;
    }
}
