
package com.github.circuitrunners;

import com.analog.adis16448.frc.ADIS16448_IMU;
import com.github.circuitrunners.calib.CalibMath;
import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

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

    private RobotDrive drive;

    private VictorSP frontLeft;
    private VictorSP frontRight;
    private VictorSP rearLeft;
    private VictorSP rearRight;

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

        frontLeft = new VictorSP(PORT_DRIVE_FRONT_LEFT);
        rearLeft = new VictorSP(PORT_DRIVE_REAR_LEFT);
        frontRight = new VictorSP(PORT_DRIVE_FRONT_RIGHT);
        rearRight = new VictorSP(PORT_DRIVE_REAR_RIGHT);

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
        SmartDashboard.putNumber("liftOffset", 0); //save "derp" to dictionary

        SmartDashboard.putNumber("pid_kP", theOtherPIDController.getP());
        SmartDashboard.putNumber("pid_kI", theOtherPIDController.getI());
        SmartDashboard.putNumber("pid_kD", theOtherPIDController.getD());
        theOtherPIDController.enable();

        weatherStatus = CalibMath.answerQuestion();
    }

    boolean triggerPressed; //so lonely

    @Override
    public void teleopPeriodic() {

        double moveVal = joystick.getY();
        double twistVal = joystick.getTwist();
        double throttleVal = -joystick.getThrottle();

        double rotateVal = CalibMath.scalePower(twistVal, 0.05, 0.7, 2); //could make magic numbers into constants but who cares
        double throttledMove = CalibMath.throttleMath(throttleVal) * moveVal; //0% chance we need this elsewhere but who cares
        //there's no code in this line but who cares

        double thisShitDegrees = Math.toDegrees(thisShit.getYaw());

        double gyroVal = thisShitDegrees - thisAdjustment;

        if (joystick.getRawButton(2)) {
            thisAdjustment = thisShitDegrees;
            thatAdjustment = thatShit.getAngle();
            theOtherAdjustment = theOtherShit.getAngle();
        }

        if (thatPIDController.isEnabled()){
            if (Math.abs(rotateVal) > 0.2) {
                thatPIDController.setSetpoint(theOtherShit.getAngle());
                rotateVal += thatPIDController.get();
            } else {
                rotateVal = thatPIDController.get();
            }
        }
        if (joystick.getRawButton(4) && !triggerPressed) {
            thatPIDController.setSetpoint(theOtherShit.getAngle());
            triggerPressed = true;
        }
        if (joystick.getRawButton(4)){
            thatPIDController.enable();
        } else {
            thatPIDController.disable();
            triggerPressed = false;
        }

        drive.arcadeDrive(throttledMove, rotateVal);

        if (joystick.getRawButton(3)) {
            shooterWheelLeft.set(1);
            shooterWheelRight.set(-1);
            Timer.delay(0.5);
            shooterKicker.set(-1);
        } else if (joystick.getRawButton(1)) {
            shooterWheelLeft.set(-1);
            shooterWheelRight.set(1);
            Timer.delay(0.5);
            shooterKicker.set(1);
        } else {
            shooterWheelLeft.set(0);
            shooterWheelRight.set(0);
            shooterKicker.set(-1);
        }

        if (joystick.getRawButton(11)) {
            shooterWheelLift.set(-1);
        } else if (joystick.getRawButton(12)) {
            shooterWheelLift.set(1);
        } else {
            shooterWheelLift.set(SmartDashboard.getNumber("liftOffset"));
        }

        SmartDashboard.putNumber("moveVal", moveVal);
        SmartDashboard.putNumber("rotateVal", rotateVal);
        SmartDashboard.putNumber("gyroVal", gyroVal);

        SmartDashboard.putBoolean("isPIDEnabled", thatPIDController.isEnabled());
        SmartDashboard.putNumber("Setpoint", thatPIDController.getSetpoint());
        double kP = SmartDashboard.getNumber("pid_kP");
        double kI = SmartDashboard.getNumber("pid_kI");
        double kD = SmartDashboard.getNumber("pid_kD");
        thatPIDController.setPID(kP, kI, kD);
        SmartDashboard.putNumber("pidValue", thatPIDController.get());

        double temp = thisShit.getTemperature();

        SmartDashboard.putNumber("derp", CalibMath.adjustedDeadband(joystick.getX(),0.3));
        SmartDashboard.putNumber("Temperature", thisShit.getTemperature());
        SmartDashboard.putNumber("Pressure", thisShit.getBarometricPressure());

        double[] gyroArray = {thisShitDegrees-thisAdjustment, thatShit.getAngle()-thatAdjustment, theOtherShit.getAngle()-theOtherAdjustment};
        double gyroAverage = CalibMath.average(gyroArray);
        SmartDashboard.putNumber("gyroAngle",CalibMath.gyroLimit(gyroAverage));

        SmartDashboard.putString("Will it rain?", weatherStatus);
    }
}
