
package com.github.circuitrunners;

import com.akilib.SmartDashboard2;
import com.analog.adis16448.frc.ADIS16448_IMU;
import com.github.circuitrunners.calib.CalibMath;
import edu.wpi.first.wpilibj.*;

public class Robot extends IterativeRobot {

    //Port Constants
    public static final int PORT_DRIVE_FRONT_LEFT = 0;
    public static final int PORT_DRIVE_REAR_LEFT = 1;
    public static final int PORT_DRIVE_FRONT_RIGHT = 2;
    public static final int PORT_DRIVE_REAR_RIGHT = 3;
    
    private static final int PORT_SHOOTER_LEFT = 4;
    private static final int PORT_SHOOTER_RIGHT = 5;
    private static final int PORT_SHOOTER_KICKER = 6;
    private static final int PORT_SHOOTER_LIFT = 0;

    //PID Constants
    public static final double THE_OTHER_SHIT_KP = 0.05;
    public static final double THE_OTHER_SHIT_KD = 0.1;
    
    private static final double THIS_SHIT_KP = 0;
    private static final double THIS_SHIT_KD = 0;
    
    private static final double THAT_SHIT_KP = 0;
    private static final double THAT_SHIT_KD = 0;
    
    // Buttons
    private static final int BUTTON_GYRO_RESET = 9;
    private static final int BUTTON_PID_ENABLE = 10;
    
    private static final int BUTTON_SHOOTER_WHEELSPIN_IN = 2;
    private static final int BUTTON_SHOOTER_WHEELSPIN_OUT = 1;
    private static final int BUTTON_SHOOTER_LIFT_UP = 5;
    private static final int BUTTON_SHOOTER_LIFT_DOWN = 3;
    private static final double OFFSET_SHOOTER = 0.17;
    private static final double SPEED_SHOOTER_LIFT_UP = -1;
    private static final double SPEED_SHOOTER_LIFT_DOWN = 1;

    private RobotDrive drive;

    private VictorSP shooterWheelLeft;
    private VictorSP shooterWheelRight;
    private Servo shooterKicker;

    private CANTalon shooterWheelLift;

    private Joystick joystick;

    private AnalogGyro theOtherShit;
    private PIDController theOtherPIDController;

    private ADIS16448_IMU thisShit;
    private PIDController thisPIDController;
    
    private ADXRS450_Gyro thatShit;
    private PIDController thatPIDController;
    private String weatherStatus;

    private double thisAdjustment;
    private double thatAdjustment;
    private double theOtherAdjustment;

    @Override
    public void robotInit() {

        VictorSP frontLeft = new VictorSP(PORT_DRIVE_FRONT_LEFT);
        VictorSP rearLeft = new VictorSP(PORT_DRIVE_REAR_LEFT);
        VictorSP frontRight = new VictorSP(PORT_DRIVE_FRONT_RIGHT);
        VictorSP rearRight = new VictorSP(PORT_DRIVE_REAR_RIGHT);

        drive = new RobotDrive(frontLeft, rearLeft, frontRight, rearRight);

        drive.setExpiration(0.5);

        shooterWheelLeft = new VictorSP(PORT_SHOOTER_LEFT);
        shooterWheelRight = new VictorSP(PORT_SHOOTER_RIGHT);
        shooterKicker = new Servo(PORT_SHOOTER_KICKER);
        shooterWheelLift = new CANTalon(PORT_SHOOTER_LIFT);

        joystick = new Joystick(0);
        
        thisShit = new ADIS16448_IMU();
        thisShit.calibrate();
        thisPIDController = new PIDController(THIS_SHIT_KP, 0, THIS_SHIT_KD, thisShit, output -> {});

        thatShit = new ADXRS450_Gyro();
        thatShit.calibrate();
        thatPIDController = new PIDController(THAT_SHIT_KP, 0, THAT_SHIT_KD, thatShit, output -> {});

        theOtherShit = new AnalogGyro(0);
        theOtherShit.calibrate();
        theOtherPIDController = new PIDController(THE_OTHER_SHIT_KP, 0, THE_OTHER_SHIT_KD, theOtherShit, output -> {});

    }

    @Override
    public void autonomousInit() {

    }

    @Override
    public void autonomousPeriodic() {

    }

    @Override
    public void teleopInit() {

        // PID Constants
        SmartDashboard2.put("thisPID_kP", thisPIDController.getP());
        SmartDashboard2.put("thisPID_kI", thisPIDController.getI());
        SmartDashboard2.put("thisPID_kD", thisPIDController.getD());

        SmartDashboard2.put("thatPID_kP", thatPIDController.getP());
        SmartDashboard2.put("thatPID_kI", thatPIDController.getI());
        SmartDashboard2.put("thatPID_kD", thatPIDController.getD());

        SmartDashboard2.put("theOtherPID_kP", theOtherPIDController.getP());
        SmartDashboard2.put("theOtherPID_kI", theOtherPIDController.getI());
        SmartDashboard2.put("theOtherPID_kD", theOtherPIDController.getD());

        // Just in case...
        thisPIDController.disable();
        thatPIDController.disable();
        theOtherPIDController.disable();

        // Shooter
        SmartDashboard2.put("liftOffset", OFFSET_SHOOTER);
        SmartDashboard2.put("kickerOffset", 0.3);

        weatherStatus = CalibMath.answerQuestion();
    }

    boolean triggerPressed; //so lonely...
    boolean flip = false; // not anymore

    @Override
    public void teleopPeriodic() {

        // Drive values
        double moveVal = joystick.getY();
        double twistVal = joystick.getTwist();
        double throttleVal = -joystick.getThrottle();

        double rotateVal = CalibMath.scalePower(twistVal, 0.1, 0.85, 2); //could make magic numbers into constants but who cares
        double throttledMove = CalibMath.throttleMath(throttleVal) * moveVal; //0% chance we need this elsewhere but who cares
        //there's no code in this line but who cares

        // Gyro values
        double thisDegrees = thisShit.getAngle() - thisAdjustment;
        double thatDegrees = thatShit.getAngle() - thatAdjustment;
        double theOtherDegrees = theOtherShit.getAngle() - theOtherAdjustment;

        // Gyro reset
        if (joystick.getRawButton(BUTTON_GYRO_RESET)) {
            thisAdjustment = thisDegrees;
            thatAdjustment = thatShit.getAngle();
            theOtherAdjustment = theOtherShit.getAngle();
        }

        rotateVal += pidAdjust(thisPIDController, thisDegrees, rotateVal);

        drive.arcadeDrive(throttledMove, rotateVal);

        liftShooter();
        shootAndIntake();

        // Debug
        // Drive values
        SmartDashboard2.put("moveVal", moveVal);
        SmartDashboard2.put("rotateVal", rotateVal);
        SmartDashboard2.put("derp", CalibMath.adjustedDeadband(joystick.getX(),0.3)); //save "derp" to dictionary

        // Gyro values
        SmartDashboard2.put("thisGyroVal", thisDegrees);
        SmartDashboard2.put("thatGyroVal", thatDegrees);
        SmartDashboard2.put("theOtherGyroVal", theOtherDegrees);

        double[] gyroArray = {thisDegrees-thisAdjustment,
                thatShit.getAngle()-thatAdjustment,
                theOtherShit.getAngle()-theOtherAdjustment};
        double gyroAverage = CalibMath.average(gyroArray);
        SmartDashboard2.put("gyroAngle",CalibMath.gyroLimit(gyroAverage));

        // PID values
        SmartDashboard2.put("isPIDEnabled", thatPIDController.isEnabled());
        SmartDashboard2.put("setpoint", thatPIDController.getSetpoint());
        double kP = SmartDashboard2.getNumber("theOtherPID_kP");
        double kI = SmartDashboard2.getNumber("theOtherPID_kI");
        double kD = SmartDashboard2.getNumber("theOtherPID_kD");
        thatPIDController.setPID(kP, kI, kD);
        SmartDashboard2.put("pidValue", thatPIDController.get());

        //Weather
        SmartDashboard2.put("Temperature", thisShit.getTemperature());
        SmartDashboard2.put("Pressure", thisShit.getBarometricPressure());
        SmartDashboard2.put("Will it rain?", weatherStatus);
    }

    private double pidAdjust(PIDController pidController, double setpoint, double rotateVal) {
        // PID Enable/Disable
        if (joystick.getRawButton(BUTTON_PID_ENABLE) && !triggerPressed) {
            pidController.setSetpoint(setpoint);
            triggerPressed = true;
        }
        if (joystick.getRawButton(BUTTON_PID_ENABLE)){
            pidController.enable();
        } else {
            thisPIDController.disable();
            triggerPressed = false;
        }

        // PID Control
        if (thisPIDController.isEnabled()){
            if (Math.abs(rotateVal) > 0.2) {
                thisPIDController.setSetpoint(setpoint);
                return rotateVal + thisPIDController.get();
            } else {
                return thisPIDController.get();
            }
        }
        return 0;
    }

    public void liftShooter() {
        // Shooter Lift
        if (joystick.getRawButton(BUTTON_SHOOTER_LIFT_UP)) {
            shooterWheelLift.set(SPEED_SHOOTER_LIFT_UP);
        } else if (joystick.getRawButton(BUTTON_SHOOTER_LIFT_DOWN)) {
            shooterWheelLift.set(SPEED_SHOOTER_LIFT_DOWN);
        } else {
            double offset = SmartDashboard2.getNumber("liftOffset", OFFSET_SHOOTER);
            offset += flip ? 0 : 0.05;
            shooterKicker.set(offset);
            flip = !flip;
        }
    }

    public void shootAndIntake() {
        // Shooter Wheels
        if (joystick.getRawButton(BUTTON_SHOOTER_WHEELSPIN_IN)) {
            shooterWheelLeft.set(1);
            shooterWheelRight.set(-1);
            shooterKicker.set(-1);
        } else if (joystick.getRawButton(BUTTON_SHOOTER_WHEELSPIN_OUT)) {
            shooterWheelLeft.set(-1);
            shooterWheelRight.set(1);
            Timer.delay(0.5); // Spinning up...
            shooterKicker.set(1);
        } else {
            shooterWheelLeft.set(0);
            shooterWheelRight.set(0);
            shooterKicker.set(-SmartDashboard2.getNumber("kickerReturn", 0.3));
        }
    }
}
