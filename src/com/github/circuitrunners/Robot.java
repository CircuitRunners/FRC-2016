
package com.github.circuitrunners;

import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.util.ArrayList;

public class Robot extends IterativeRobot {

    Joystick joystick;

    AnalogGyro gyro;

    CANTalon motors[];

    ArrayList<PIDController> pids;

    @Override
    public void robotInit() {
        joystick = new Joystick(0);
        gyro = new AnalogGyro(0);
        motors = new CANTalon[]{new CANTalon(0), new CANTalon(3), new CANTalon(4), new CANTalon(5)};
        pids = new ArrayList<>();
        for (CANTalon motor : motors) {
            pids.add(new PIDController(1, 1, 1, 1, gyro, motor));
        }
        motors[0].reverseOutput(true);
        motors[3].reverseOutput(true);
    }

    @Override
    public void autonomousInit() {

    }

    @Override
    public void autonomousPeriodic() {

    }

    @Override
    public void teleopInit() {

    }

    private boolean isGyroControlEnabled;
    @Override
    public void teleopPeriodic() {
        double moveVal = joystick.getY();
        double rotateVal = joystick.getTwist();

        double gyroVal = gyro.getAngle();
        if (joystick.getRawButton(1)) {
            gyro.reset();
        }

        if (joystick.getRawButton(2)) {
            isGyroControlEnabled = !isGyroControlEnabled;
        }

        if (isGyroControlEnabled){
            if (gyroVal > 3){
                rotateVal = -0.15;
            } else if (gyroVal < 177) {
                rotateVal = 0.15;
            }
        }
        double atan = Math.atan2(joystick.getY(),joystick.getX());
        if (0 <= atan && atan < 90) {
            motors[0].set(Math.sin(atan));
            motors[3].set(Math.sin(atan));
            motors[4].set(1);
            motors[5].set(1);
        } else if (90 <= atan && atan < 180) {
            motors[4].set(Math.sin(atan));
            motors[5].set(Math.sin(atan));
            motors[0].set(1);
            motors[3].set(1);
        } else if (180 <= atan && atan < 270){
            motors[4].set(Math.sin(atan));
            motors[5].set(Math.sin(atan));
            motors[0].set(-1);
            motors[3].set(-1);
        } else if (270 <= atan && atan < 360){
            motors[0].set(Math.sin(atan));
            motors[3].set(Math.sin(atan));
            motors[4].set(-1);
            motors[5].set(-1);
        } else {
            for (CANTalon motor : motors) {
                motor.set(0);
            }
        }
        SmartDashboard.putNumber("moveVal", moveVal);
        SmartDashboard.putNumber("rotateVal", rotateVal);
        SmartDashboard.putNumber("gyroVal", gyroVal);
        SmartDashboard.putData("test", gyro);
    }

    @Override
    public void testPeriodic() {

    }

}
