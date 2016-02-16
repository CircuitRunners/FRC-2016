
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

    // Drive Adjustments
    private static final double JOYSTICK_DEADZONE = 0.1;
    private static final double JOYSTICK_SCALE_FLAT = 0.85;
    private static final double JOYSTICK_SCALE_POWER = 2;
    private static final double JOYSTICK_DEADZONE_PID = 0.2;



    // Axes
    private static int AXIS_MOVE = 1;
    private static int AXIS_ROTATE = 2;
    private static int AXIS_THROTTLE = 3;

    // Buttons
    private static int BUTTON_GYRO_RESET = 9;
    private static int BUTTON_PID_ENABLE = 10;

    private static int BUTTON_SHOOTER_WHEELSPIN_IN = 2;
    private static int BUTTON_SHOOTER_WHEELSPIN_OUT = 1;
    private static int BUTTON_SHOOTER_LIFT_UP = 5;
    private static int BUTTON_SHOOTER_LIFT_DOWN = 3;
    private static final double OFFSET_SHOOTER = 0.17;
    private static final double SPEED_SHOOTER_LIFT_UP = -0.5;
    private static final double SPEED_SHOOTER_LIFT_DOWN = 0.5;

    //PID Constants
    private double KP_POT = 0;
    private double KI_POT = 0;
    private double KD_POT = 0;

    private static final double KP_THIS_SHIT = 0;
    private static final double KD_THIS_SHIT = 0;
    
    private static final double KP_THAT_SHIT = 0;
    private static final double KD_THAT_SHIT = 0;

    public static final double KP_THE_OTHER_SHIT = 0.05;
    public static final double KD_THE_OTHER_SHIT = 0.1;
    private static final double SPEED_SHOOTER_WHEEL_LEFT = 1;
    private static final double SPEED_SHOOTER_WHEEL_RIGHT = 1;


    private RobotDrive drive;

    private VictorSP shooterWheelLeft;
    private VictorSP shooterWheelRight;
    private Servo shooterKicker;

    private CANTalon shooterWheelLift;

    private Joystick joystick;

    private AnalogGyro theOtherShit;
    private PIDController theOtherPIDController;

    private ADIS16448_IMU thisShit;
    //private PIDController thisPIDController;
    
    private ADXRS450_Gyro thatShit;
    private PIDController thatPIDController;
    private String weatherStatus;

    private DigitalInput hall1;
    private AnalogPotentiometer pot;

    //private double thisAdjustment;
    private double thatAdjustment;
    private double theOtherAdjustment;

    private double angleCoeff = 0.17;
    private double targetAngle = 0;
    private PIDController potPID;

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

        hall1 = new DigitalInput(0);
        pot = new AnalogPotentiometer(1,5000); // Range: 1950-4560
        potPID = new PIDController(KP_POT, KI_POT, KD_POT, pot, shooterWheelLift);

        joystick = new Joystick(0);
        
        thisShit = new ADIS16448_IMU();
        thisShit.calibrate();
//        thisPIDController = new PIDController(KP_THIS_SHIT, 0, KD_THIS_SHIT, thisShit, output -> {});

        thatShit = new ADXRS450_Gyro();
        thatShit.calibrate();
        thatPIDController = new PIDController(KP_THAT_SHIT, 0, KD_THAT_SHIT, thatShit, output -> {});

        theOtherShit = new AnalogGyro(0);
        theOtherShit.calibrate();
        theOtherPIDController = new PIDController(KP_THE_OTHER_SHIT, 0, KD_THE_OTHER_SHIT, theOtherShit, output -> {});

    }

    @Override
    public void autonomousInit() {

    }

    @Override
    public void autonomousPeriodic() {

    }

    @Override
    public void teleopInit() {

        // Reset adjustment
//        thisAdjustment = thisShit.getAngle();

        // Just in case...
//        thisPIDController.disable();
        thatPIDController.disable();
        theOtherPIDController.disable();

        potPID.enable();

        weatherStatus = CalibMath.answerQuestion();

        // Drive Controls
        if (SmartDashboard2.get("isXbox", false)) {
            AXIS_MOVE = 1;
            AXIS_ROTATE = 4;

            BUTTON_GYRO_RESET = 6;
            BUTTON_PID_ENABLE = 7;

            BUTTON_SHOOTER_LIFT_DOWN = 1;
            BUTTON_SHOOTER_LIFT_UP = 2;
            BUTTON_SHOOTER_WHEELSPIN_OUT = 6;
            BUTTON_SHOOTER_WHEELSPIN_IN = 5;
        }
    }

    boolean triggerPressed; //so lonely

    @Override
    public void teleopPeriodic() {

        // Drive values
        double moveVal = joystick.getRawAxis(AXIS_MOVE);
        double twistVal = joystick.getRawAxis(AXIS_ROTATE);
        double throttleVal = CalibMath.throttleMath(-joystick.getRawAxis(AXIS_THROTTLE));

        if (SmartDashboard2.get("isXbox", false)) {
            throttleVal = 0;
        }

        double rotateVal = CalibMath.scalePower(twistVal, JOYSTICK_DEADZONE, JOYSTICK_SCALE_FLAT, JOYSTICK_SCALE_POWER); //could make magic numbers into constants but who cares
        double throttledMove = (throttleVal) * moveVal; //0% chance we need this elsewhere but who cares
        double throttledRotate = CalibMath.inverseAdjustedDeadband(throttleVal,0.5) * rotateVal;//there's no code in this line but who cares

        // Gyro values
//        double thisRadians = thisShit.getAngle();
//        double thisDegrees = Math.toDegrees(thisRadians);
        double thatDegrees = thatShit.getAngle();
        double theOtherDegrees = theOtherShit.getAngle();
//        double thisAdjusted = thisDegrees - thisAdjustment;
        double thatAdjusted = thatDegrees - thatAdjustment;
        double theOtherAdjusted = theOtherDegrees - theOtherAdjustment;

        // Gyro reset
        if (joystick.getRawButton(BUTTON_GYRO_RESET)) {
//            thisAdjustment += thisDegrees;
            thatAdjustment += thatDegrees;
            theOtherAdjustment += theOtherDegrees;

        }
        double angle = (pot.get() - 1950) / 17.4;

        throttledRotate += pidAdjust(thatPIDController, thatDegrees, rotateVal);

        drive.arcadeDrive(throttledMove, throttledRotate);

        liftShooter();
        shootAndIntake();

        // Debug
        // Drive values
        SmartDashboard2.put("moveVal", moveVal);
        SmartDashboard2.put("rotateVal", rotateVal);
        SmartDashboard2.put("throttledRotate", throttledRotate);
        SmartDashboard2.put("derp", CalibMath.adjustedDeadband(joystick.getX(),0.3)); //save "derp" to dictionary
        SmartDashboard2.put("angle", angle);

        // Gyro values
//        SmartDashboard2.put("thisGyroVal", thisRadians);
        SmartDashboard2.put("thatGyroVal", thatDegrees);
        SmartDashboard2.put("theOtherGyroVal", theOtherDegrees);

        // PID values
        pidControl(thatPIDController, "that");
        pidControl(theOtherPIDController, "theOther");

        //Weather
        SmartDashboard2.put("Temperature", thisShit.getTemperature());
        SmartDashboard2.put("Pressure", thisShit.getBarometricPressure());
        SmartDashboard2.put("Will it rain?", weatherStatus);

        //Smart Gyro™
        double gyroAverage = CalibMath.average(CalibMath.gyroLimit(thatAdjusted),CalibMath.gyroLimit(theOtherAdjusted));
        SmartDashboard2.put("SmartGyro",gyroAverage);

        SmartDashboard2.put("Hall",hall1);
        SmartDashboard2.put("Pot", pot.get());

        SmartDashboard2.get("kp_pot", KP_POT);
        SmartDashboard2.get("ki_pot", KI_POT);
        SmartDashboard2.get("kd_pot", KD_POT);
        potPID.setPID(KP_POT,KI_POT,KD_POT);

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
            pidController.disable();
            triggerPressed = false;
        }

        // PID Control
        if (pidController.isEnabled()){
            if (Math.abs(rotateVal) > JOYSTICK_DEADZONE_PID) {
                pidController.setSetpoint(setpoint);
                return rotateVal + pidController.get();
            } else {
                return pidController.get();
            }
        }
        return 0;
    }

    public void liftShooter() {
        double liftUpSpeed = SmartDashboard2.get("liftUpSpeed", SPEED_SHOOTER_LIFT_UP);
        double liftDownSpeed = SmartDashboard2.get("liftDownSpeed", SPEED_SHOOTER_LIFT_DOWN);
        // Shooter Lift
        if (joystick.getRawButton(BUTTON_SHOOTER_LIFT_UP)) {
            SmartDashboard2.put("targetAngle", ++targetAngle);
        } else if (joystick.getRawButton(BUTTON_SHOOTER_LIFT_DOWN)) {
            SmartDashboard2.put("targetAngle", --targetAngle);
        }
        targetAngle = SmartDashboard2.get("targetAngle", targetAngle);
        double setpoint = targetAngle * 17.4 + 1950;
        potPID.setSetpoint(setpoint);
        SmartDashboard2.put("targetAngle", targetAngle);
        System.out.println(targetAngle);
    }

    public void shootAndIntake() {
        double shooterLeftWheelSpeed = SmartDashboard2.get("leftWheelSpeed", SPEED_SHOOTER_WHEEL_LEFT);
        double shooterRightWheelSpeed = SmartDashboard2.get("rightWheelSpeed", SPEED_SHOOTER_WHEEL_RIGHT);
        // Shooter Wheels
        if (joystick.getRawButton(BUTTON_SHOOTER_WHEELSPIN_IN)) {
            shooterWheelLeft.set(shooterLeftWheelSpeed);
            shooterWheelRight.set(-shooterRightWheelSpeed);
            shooterKicker.set(-1);
        } else if (joystick.getRawButton(BUTTON_SHOOTER_WHEELSPIN_OUT)) {
            shooterWheelLeft.set(-shooterLeftWheelSpeed);
            shooterWheelRight.set(shooterRightWheelSpeed);
            Timer.delay(0.5); // Spinning up...
            shooterKicker.set(1);
        } else {
            shooterWheelLeft.set(0);
            shooterWheelRight.set(0);
            shooterKicker.set(-SmartDashboard2.get("kickerReturn", 0.3));
        }
    }
    
    private void pidControl(PIDController pidController, String name) {
        SmartDashboard2.put("is" + name + "PIDEnabled", pidController.isEnabled());
        SmartDashboard2.put(name + "Setpoint", pidController.getSetpoint());
        double kP = SmartDashboard2.get(name + "PID_kP", pidController.getP());
        double kI = SmartDashboard2.get(name + "PID_kI", pidController.getI());
        double kD = SmartDashboard2.get(name + "PID_kD", pidController.getD());
        pidController.setPID(kP, kI, kD);
        SmartDashboard2.put(name + "PIDValue", pidController.get());
    }
}
