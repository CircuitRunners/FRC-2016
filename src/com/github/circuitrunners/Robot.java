
package com.github.circuitrunners;

import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Robot extends IterativeRobot {

    VictorSP leftESC, rightESC;
    RobotDrive drive;

    Joystick xbox;

    AnalogGyro gyro;

    PIDController leftPID, rightPID;

    @Override
    public void robotInit() {

        leftESC = new VictorSP(0);
        rightESC = new VictorSP(1);
        drive = new RobotDrive(leftESC, rightESC);

        xbox = new Joystick(0);

        gyro = new AnalogGyro(0);

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
        double moveVal = xbox.getY(Joystick.Hand.kLeft);
        double rotateVal = xbox.getX(Joystick.Hand.kRight);

        double gyroVal = gyro.getAngle();
        if (xbox.getRawButton(1)) gyro.reset();

        if (xbox.getRawButton(2)) isGyroControlEnabled = !isGyroControlEnabled;
        if (isGyroControlEnabled){
            if (gyroVal > 3){
                rotateVal = -0.15;
            } else if (gyroVal < 177) {
                rotateVal = 0.15;
            }
        }

        drive.tankDrive(moveVal, rotateVal);

        SmartDashboard.putNumber("moveVal", moveVal);
        SmartDashboard.putNumber("rotateVal", rotateVal);
        SmartDashboard.putNumber("gyroVal", gyroVal);

        SmartDashboard.putData("test", gyro);
    }

    @Override
    public void testPeriodic() {

    }

}
