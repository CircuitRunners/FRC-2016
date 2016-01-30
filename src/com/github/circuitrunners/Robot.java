
package com.github.circuitrunners;

import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;

public class Robot extends IterativeRobot {

    private RobotDrive drive;
    private Joystick joystick;
    private AnalogGyro gyro;

    private static final ExecutorService parallelExecutor = Executors.newCachedThreadPool();
    private static final ExecutorService sequentialExecutor = Executors.newSingleThreadExecutor();

    @Override
    public void robotInit() {
        drive = new RobotDrive(5, 4, 1, 0);
        drive.setExpiration(0.5);

        joystick = new Joystick(0);

        gyro = new AnalogGyro(0);
        gyro.calibrate();
    }

    @Override
    public void autonomousInit() {
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
        drive(-joystick.getY(), -joystick.getX(), true);
    }

    private void drive(double moveVal, double rotateVal) {
        drive.arcadeDrive(moveVal, rotateVal);

        SmartDashboard.putNumber("moveVal", moveVal);
        SmartDashboard.putNumber("rotateVal", rotateVal);
        SmartDashboard.putNumber("gyro", gyro.getAngle());
    }

    private void drive(double moveVal, double rotateVal, boolean input) {
        if (input) {
            double throttleVal = -joystick.getThrottle();

            moveVal *= throttleMath(throttleVal);
            rotateVal = scalePower(rotateVal, 0.1, 0.7, 2);
        }

        drive(moveVal, rotateVal);
    }

    private double scaleLinear(double input, double min, double scale) {
        double output = 0;
        if (Math.abs(input) >= min) {
            output = scale * input;
        }
        return output;
    }

    private double scaleDoubleFlat(double input, double min, double scale, double power) {
        double output;
        if (Math.abs(input) < min) {
            output = 0;
        } else if (Math.abs(input) > scale) {
            output = Math.signum(input) * scale;
        } else {
            output = scale * Math.signum(input) * Math.abs(Math.pow(input/scale,power));
        }
        return output;
    }

    private double scalePower(double input, double min, double scale, double power) {
        double output = 0;
        if (Math.abs(input) >= min) {
            output = scale * Math.signum(input) * Math.abs(Math.pow(input,power));
        }
        return output;
    }

    private double throttleMath(double input) {
        return (input + 1) / 2;
    }
}
