
package com.github.circuitrunners;

import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Robot extends IterativeRobot {

    RobotDrive drive;

    VictorSP motor;

    Joystick joystick;

    AnalogGyro gyro;

    PIDController pidController;

    @Override
    public void robotInit() {

        drive = new RobotDrive(5,4,1,0);
        motor = new VictorSP(3);

        joystick = new Joystick(0);

        gyro = new AnalogGyro(0);

        pidController = new PIDController(0, 0, 0, gyro, output -> {});

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
        SmartDashboard.putNumber("derp", 0);

        SmartDashboard.putNumber("pid_kP", pidController.getP());
        SmartDashboard.putNumber("pid_kI", pidController.getI());
        SmartDashboard.putNumber("pid_kD", pidController.getD());
        pidController.enable();
    }
    boolean triggerPressed;
    @Override
    public void teleopPeriodic() {
        double moveVal = -joystick.getY(); // wtf are directions
        double twistVal = -joystick.getTwist(); //jesus cant save you now
        double throttleVal = -joystick.getThrottle(); //why is everything negative?

        double rotateVal = scalePower(twistVal, 0.1, 0.7, 2); //could make magic numbers into constants but who cares
        double throttledMove = throttleMath(throttleVal) * moveVal; //0% chance we need this elsewhere but who cares
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
        if (joystick.getRawButton(4)){
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
