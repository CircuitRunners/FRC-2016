
package com.github.circuitrunners;

import edu.wpi.first.wpilibj.Gyro;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.RobotDrive;

public class Robot extends IterativeRobot {

    Gyro gyro;
    RobotDrive drive;
    Joystick joystickLeft;
    Joystick joystickRight;

    @Override
    public void robotInit() {
        gyro = new Gyro(0);
        drive = new RobotDrive(0, 1);
        joystickLeft = new Joystick(0);
        joystickRight = new Joystick(1);
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

    private boolean button;
    @Override
    public void teleopPeriodic() {
        double leftVal = joystickLeft.getY();
        double rightVal = joystickRight.getY();
        double gyroVal = gyro.getAngle();
        if (joystickRight.getRawButton(4)){
            button = !button;
        }
        if (joystickRight.getRawButton(2)){
            gyro.reset();
        }
        if (button){
            if (gyroVal>180){
                rightVal *= 0.95;
            } else if (gyroVal<180) {
                leftVal *= 0.95;
            }
        }
        drive.tankDrive(leftVal, rightVal);
    }

    @Override
    public void testPeriodic() {
    
    }
    
}
