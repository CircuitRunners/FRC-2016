
package com.github.circuitrunners;

import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Robot extends IterativeRobot {

    public static final double KP = 0.05;
    public static final double KD = 0.1;
    RobotDrive drive;

    VictorSP frontLeft;
    VictorSP frontRight;
    VictorSP rearLeft;
    VictorSP rearRight;

    VictorSP motor;

    Joystick joystick;

    AnalogGyro gyro;

    PIDController pidController;

    Thread driveThread;

    @Override
    public void robotInit() {

        frontLeft = new VictorSP(0);
        frontRight = new VictorSP(5);
        rearLeft = new VictorSP(1);
        rearRight = new VictorSP(4);

        drive = new RobotDrive(frontLeft,rearLeft,frontRight,rearRight);
        motor = new VictorSP(3);

        joystick = new Joystick(0);

        gyro = new AnalogGyro(0);
        gyro.calibrate();

        pidController = new PIDController(KP, 0,  KD, gyro, output -> {});


        driveThread = new Thread();

        drive.setExpiration(0.5);

    }

    @Override
    public void autonomousInit() {

    }

    @Override
    public void autonomousPeriodic() {

    }

    @Override
    public void teleopInit() {
        SmartDashboard.putNumber("derp", 0); //save "derp" to dictionary

        SmartDashboard.putNumber("pid_kP", pidController.getP());
        SmartDashboard.putNumber("pid_kI", pidController.getI());
        SmartDashboard.putNumber("pid_kD", pidController.getD());
        pidController.enable();
    }

    boolean triggerPressed; //so lonely

    @Override
    public void teleopPeriodic() {

        double moveVal = -joystick.getY();
        double twistVal = joystick.getTwist();
        double throttleVal = -joystick.getThrottle();

        double rotateVal = CalibMath.scalePower(twistVal, 0.1, 0.7, 2); //could make magic numbers into constants but who cares
        double throttledMove = CalibMath.throttleMath(throttleVal) * moveVal; //0% chance we need this elsewhere but who cares
        //there's no code in this line but who cares
        double gyroVal = gyro.getAngle();
        if (joystick.getRawButton(2)) gyro.reset();
        if (pidController.isEnabled()){
            if (Math.abs(rotateVal) > 0.2) {
                pidController.setSetpoint(gyro.getAngle());
                rotateVal += pidController.get();
            } else {
                rotateVal = pidController.get();
            }
        }
        if (joystick.getRawButton(4) && !triggerPressed) {
            pidController.setSetpoint(gyro.getAngle());
            triggerPressed = true;
        }
        if (joystick.getRawButton(4)||joystick.getRawButton(3)){
            pidController.enable();
        } else {
            pidController.disable();
            triggerPressed = false;
        }

        drive.arcadeDrive(throttledMove, rotateVal);

        SmartDashboard.putNumber("moveVal", moveVal);
        SmartDashboard.putNumber("rotateVal", rotateVal);
        SmartDashboard.putNumber("gyroVal", gyroVal);

        SmartDashboard.putBoolean("isPIDEnabled", pidController.isEnabled());
        SmartDashboard.putNumber("Setpoint", pidController.getSetpoint());
        double kP = SmartDashboard.getNumber("pid_kP");
        double kI = SmartDashboard.getNumber("pid_kI");
        double kD = SmartDashboard.getNumber("pid_kD");
        pidController.setPID(kP, kI, kD);
        SmartDashboard.putNumber("pidError", pidController.getError());
        SmartDashboard.putNumber("pidValue", pidController.get());

        //motor.set(SmartDashboard.getNumber("derp",0));
        SmartDashboard.putNumber("derp",CalibMath.adjustedDeadband(joystick.getX(),0.3));
    }
}
