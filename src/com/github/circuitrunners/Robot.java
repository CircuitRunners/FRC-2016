
package com.github.circuitrunners;

import edu.wpi.first.wpilibj.AnalogGyro;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.RobotDrive;

import java.lang.reflect.Constructor;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;

public class Robot extends IterativeRobot {

    Joystick joystick;

    AnalogGyro gyro;

    RobotDrive drive;

    @Override
    public void robotInit() {
        joystick = new Joystick(0);
        gyro = new AnalogGyro(0);
        drive = new RobotDrive(0, 1, 2, 3);
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

    }

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

        drive.arcadeDrive(moveVal, rotateVal);
    }

    @Override
    public void testPeriodic() {

    }

}
