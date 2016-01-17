
package com.github.circuitrunners;

import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Robot extends IterativeRobot {

    RobotDrive drive;

    VictorSP motor;

    Joystick joystick;

    AnalogGyro gyro;

    PIDController pidController;

    @Override
    public void robotInit() {

        drive = new RobotDrive(0, 3, 4, 5);
        motor = new VictorSP(1);

        joystick = new Joystick(0);

        gyro = new AnalogGyro(0);

        pidController = new PIDController(0, 0, 0, gyro, new PIDDummy());

        LiveWindow.addActuator("Drive", "test", motor);
        LiveWindow.addSensor("Drive", "gyro", gyro);
        LiveWindow.addActuator("Drive", "pid", pidController);

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

        SmartDashboard.putNumber("pid_kP", pidController.getP());
        SmartDashboard.putNumber("pid_kI", pidController.getI());
        SmartDashboard.putNumber("pid_kD", pidController.getD());
        SmartDashboard.putNumber("pid_kF", pidController.getF());
    }
    @Override
    public void teleopPeriodic() {
        double moveVal = joystick.getY();
        double rotateVal = joystick.getTwist();

        double gyroVal = gyro.getAngle();
        if (joystick.getRawButton(2)) gyro.reset();
        if (joystick.getRawButton(4)) {
            if (pidController.isEnabled()) pidController.disable();
            else pidController.enable();
        }
        if (pidController.isEnabled()){
            rotateVal = pidController.get();
            pidController.reset();
        }

        drive.arcadeDrive(moveVal, rotateVal);

        SmartDashboard.putNumber("moveVal", moveVal);
        SmartDashboard.putNumber("rotateVal", rotateVal);
        SmartDashboard.putNumber("gyroVal", gyroVal);

        SmartDashboard.putBoolean("isPIDEnabled", pidController.isEnabled());
        double kP = SmartDashboard.getNumber("pid_kP");
        double kI = SmartDashboard.getNumber("pid_kI");
        double kD = SmartDashboard.getNumber("pid_kD");
        double kF = SmartDashboard.getNumber("pid_kF");
        pidController.setPID(kP, kI, kD, kF);
        SmartDashboard.putNumber("pidError", pidController.getError());
        SmartDashboard.putNumber("pidValue", pidController.get());

        motor.set(SmartDashboard.getNumber("derp", 0));
    }

    @Override
    public void testPeriodic() {}

    private class PIDDummy implements PIDOutput {
        public void pidWrite(double output) {}
    }
}
