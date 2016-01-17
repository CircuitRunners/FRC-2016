
package com.github.circuitrunners;

import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Robot extends IterativeRobot {

    RobotDrive drive;

    VictorSP motor;

    Joystick joystick;

    AnalogGyro gyro;
    double extra;

    @Override
    public void robotInit() {

        drive = new RobotDrive(0, 3, 4, 5);
        motor = new VictorSP(1);

        joystick = new Joystick(0);

        gyro = new AnalogGyro(0);


        LiveWindow.addActuator("Drive", "test", motor);
        LiveWindow.addSensor("Drive", "gyro", gyro);

    }

    @Override
    public void autonomousInit() {

    }

    @Override
    public void autonomousPeriodic() {

    }

    @Override
    public void teleopInit() {
        SmartDashboard.putNumber("derp", 0);
    }

    private boolean isGyroControlEnabled;
    @Override
    public void teleopPeriodic() {
        double moveVal = joystick.getY();
        double rotateVal = joystick.getTwist();

        double gyroVal = gyro.getAngle();
        if (joystick.getRawButton(1)) gyro.reset();

        if (joystick.getRawButton(2)) isGyroControlEnabled = !isGyroControlEnabled;
        if (isGyroControlEnabled){
            if (gyroVal > 3){
                rotateVal = -0.15;
            } else if (gyroVal < 177) {
                rotateVal = 0.15;
            }
        }

        drive.arcadeDrive(moveVal, rotateVal);


        SmartDashboard.putNumber("moveVal", moveVal);
        SmartDashboard.putNumber("rotateVal", rotateVal);
        SmartDashboard.putNumber("gyroVal", gyroVal);

        motor.set(SmartDashboard.getNumber("derp", 0));
    }

    @Override
    public void testPeriodic() {}



}
