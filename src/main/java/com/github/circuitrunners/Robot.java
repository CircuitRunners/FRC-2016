
package com.github.circuitrunners;

import com.akilib.SmartDashboard2;
import com.analog.adis16448.frc.ADIS16448_IMU;
import com.github.circuitrunners.calib.CalibMath;
import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.networktables.NetworkTable;

public class Robot extends IterativeRobot {

    //Port Constants
    public static final int PORT_DRIVE_FRONT_LEFT = 0;
    public static final int PORT_DRIVE_REAR_LEFT = 1;
    public static final int PORT_DRIVE_FRONT_RIGHT = 2;
    public static final int PORT_DRIVE_REAR_RIGHT = 3;
    
    private static final int PORT_SHOOTER_LEFT = 4;
    private static final int PORT_SHOOTER_RIGHT = 5;
    private static final int PORT_SHOOTER_LIFT = 0;
    private static final int PORT_SHOOTER_KICKER = 1;

    // Drive Adjustments
    private static final double JOYSTICK_DEADZONE = 0.1;
    private static final double JOYSTICK_SCALE_FLAT = 0.7;
    private static final double JOYSTICK_SCALE_POWER = 1;
    private static final double JOYSTICK_DEADZONE_PID = 0.2;
    private static final double DISABLEDPIDLIFTSPEEDMULTIPLIERCALLMEKYLE = 0.3;
    private static final double SPEED_SHOOTER_KICKER_OUT = 0.5;
    private static final double SPEED_SHOOTER_KICKER_IN = 0.1;

    // Axes
    private static int AXIS_MOVE = 1;
    private static int AXIS_ROTATE = 2;
    private static int AXIS_THROTTLE = 3;

    // Buttons
    private static int BUTTON_PID_ENABLE = 4;
    private static int BUTTON_GYRO_RESET = 5;

    private static int BUTTON_SHOOTER_WHEELSPIN_OUT = 1;
    private static int BUTTON_SHOOTER_WHEELSPIN_IN = 2;
    private static int BUTTON_SHOOTER_LIFT_DOWN = 3;
    private static int BUTTON_SHOOTER_LIFT_UP = 5;

    //PID Constants
    private static final double KP_POT = 0.002;
    private static final double KI_POT = 0.001;
    private static final double KD_POT = 0.001;

    private static final double KP_THIS_SHIT = 0;
    private static final double KD_THIS_SHIT = 0;
    
    private static final double KP_THAT_SHIT = 0.1;
    private static final double KD_THAT_SHIT = 0.1;

    public static final double KP_THE_OTHER_SHIT = 0.05;
    public static final double KD_THE_OTHER_SHIT = 0.1;

    //Shooter Constants
    private static final double SPEED_SHOOTER_WHEEL_LEFT = 1;
    private static final double SPEED_SHOOTER_WHEEL_RIGHT = 1;

    private static final double ANGLE_LIFT_DEFAULT = 60;
    private static final double TOLERANCE_PID_POT = 0.1;
//    private static final double OFFSET_SHOOTER = 0.17;
//    private static final double SPEED_SHOOTER_LIFT_UP = -0.5;
//    private static final double SPEED_SHOOTER_LIFT_DOWN = 0.5;

    private static final double ANGLE_SHOOTER_KICKER1_REST = 0.75;
    private static final double ANGLE_SHOOTER_KICKER2_REST = 0;
    private static final double ANGLE_SHOOTER_KICKER1_LAUNCH = 0.25;
    private static final double ANGLE_SHOOTER_KICKER2_LAUNCH = 0.5;


    private RobotDrive drive;

    private VictorSP shooterWheelLeft;
    private VictorSP shooterWheelRight;

    private CANTalon shooterWheelLift;

    private CANTalon shooterKicker;
    private DigitalInput shooterKickerReturnSensor;

    private Joystick joystick;

    private AnalogGyro theOtherShit;
    private PIDController theOtherPIDController;

    private ADIS16448_IMU thisShit;
    //private PIDController thisPIDController;
    
    private ADXRS450_Gyro thatShit;
    private PIDController thatPIDController;
    private String weatherStatus;

    private AnalogPotentiometer pot;

    //private double thisAdjustment;
    private double thatAdjustment;
    private double theOtherAdjustment;
    private PIDController potPID;
    private double[] imageValues;
    private double targetAngle;

    @Override
    public void robotInit() {

        VictorSP frontLeft = new VictorSP(PORT_DRIVE_FRONT_LEFT);
        VictorSP rearLeft = new VictorSP(PORT_DRIVE_REAR_LEFT);
        VictorSP frontRight = new VictorSP(PORT_DRIVE_FRONT_RIGHT);
        VictorSP rearRight = new VictorSP(PORT_DRIVE_REAR_RIGHT);

        drive = new RobotDrive(frontLeft, rearLeft, frontRight, rearRight);

        drive.setExpiration(0.75);

        shooterWheelLeft = new VictorSP(PORT_SHOOTER_LEFT);
        shooterWheelRight = new VictorSP(PORT_SHOOTER_RIGHT);
        shooterWheelLift = new CANTalon(PORT_SHOOTER_LIFT);
        shooterKicker = new CANTalon(PORT_SHOOTER_KICKER);
        shooterKickerReturnSensor = new DigitalInput(0);

        pot = new AnalogPotentiometer(1,5000); // Range: 1950-4560
        potPID = new PIDController(KP_POT, KI_POT, KD_POT, pot, shooterWheelLift);
        potPID.setPercentTolerance(TOLERANCE_PID_POT);
        potPID.enable();

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

        weatherStatus = CalibMath.answerQuestion();

        // Drive Controls
        if (SmartDashboard2.get("isXbox", joystick.getIsXbox())) {
            AXIS_MOVE = 1;
            AXIS_ROTATE = 4;

            BUTTON_GYRO_RESET = 6;
            BUTTON_PID_ENABLE = 7;

            BUTTON_SHOOTER_LIFT_DOWN = 1;
            BUTTON_SHOOTER_LIFT_UP = 2;
            BUTTON_SHOOTER_WHEELSPIN_OUT = 6;
            BUTTON_SHOOTER_WHEELSPIN_IN = 5;
        }
        targetAngle = SmartDashboard2.get("targetAngle", ANGLE_LIFT_DEFAULT);
        NetworkTable.getTable("SmartDashboard").globalDeleteAll();
        NetworkTable.getTable("SmartDashboard").flush();
    }

    boolean triggerPressed; //so lonely

    @Override
    public void teleopPeriodic() {

        drive();
        liftShooter();
        shootAndIntake();

        // Debug
        // Drive values
        SmartDashboard2.put("derp", CalibMath.adjustedDeadband(joystick.getX(),0.3)); //save "derp" to dictionary
        SmartDashboard2.put("angle", (pot.get() - 1950) / 17.4);
        // PID values
        SmartDashboard2.put("thatPIDController", thatPIDController);
        SmartDashboard2.put("theOtherPIDController", theOtherPIDController);

        //Weather
        SmartDashboard2.put("Temperature", thisShit.getTemperature());
        SmartDashboard2.put("Pressure", thisShit.getBarometricPressure());
        SmartDashboard2.put("Will it rain?", weatherStatus);

        SmartDashboard2.put("Hall", shooterKickerReturnSensor);
        SmartDashboard2.put("PotPID", potPID);
        boolean isShooterAimed = SmartDashboard2.get("distance", 99999) < 500 &&
                                 SmartDashboard2.get("azimuth", 99999) < 50;
        SmartDashboard2.put("goodToGo", isShooterAimed);
    }

    private void drive() {
        // Drive values
        double moveVal = SmartDashboard2.put("moveVal", joystick.getRawAxis(AXIS_MOVE));
        double twistVal = SmartDashboard2.put("twistVal", joystick.getRawAxis(AXIS_ROTATE));
        double throttleVal = SmartDashboard2.put("throttleVal",
                                                 CalibMath.throttleMath(-joystick.getRawAxis(AXIS_THROTTLE)));

        if (SmartDashboard2.get("isXbox", false)) throttleVal = 1;

        double rotateVal = SmartDashboard2.put("rotateVal", CalibMath.scaleDoubleFlat(twistVal,
                                                                                 SmartDashboard2.get("joystickDeadzone", JOYSTICK_DEADZONE),
                                                                                 SmartDashboard2.get("joystickScaleFlat", JOYSTICK_SCALE_FLAT),
                                                                                 SmartDashboard2.get("joystickScalePower", JOYSTICK_SCALE_POWER))); //could make magic numbers into constants but who cares
        double throttledMove = SmartDashboard2.put("throttledMove", (throttleVal) * moveVal); //0% chance we need this elsewhere but who cares
        //there's no code in this line but who cares

        // Gyro values
//        double thisRadians = thisShit.getAngle();
//        double thisDegrees = Math.toDegrees(thisRadians);
        double thatDegrees = SmartDashboard2.put("thatDegrees", thatShit.getAngle());
        double theOtherDegrees = SmartDashboard2.put("theOtherDegrees", theOtherShit.getAngle());
//        double thisAdjusted = thisDegrees - thisAdjustment;
        double thatAdjusted = SmartDashboard2.put("thatAdjusted", thatDegrees - thatAdjustment);
        double theOtherAdjusted = SmartDashboard2.put("theOtherAdjusted", theOtherDegrees - theOtherAdjustment);

        // Gyro reset
        if (joystick.getRawButton(BUTTON_GYRO_RESET)) {
//            thisAdjustment += thisDegrees;
            thatAdjustment += thatDegrees;
            theOtherAdjustment += theOtherDegrees;

        }

        //Smart Gyro™
        SmartDashboard2.put("SmartGyro", CalibMath.average(CalibMath.gyroLimit(thatAdjusted),
                                                           CalibMath.gyroLimit(theOtherAdjusted)));

        double throttledRotate = SmartDashboard2.put("throttledRotate",
                                                     CalibMath.inverseAdjustedDeadband(throttleVal,0.5) * rotateVal
                                                     + pidAdjust(thatPIDController, thatDegrees, rotateVal));

        if (Math.abs(throttledMove) > 0.2 || SmartDashboard2.get("targetAngle", 110) > 110) {
            potPID.disable();
        } else {
            potPID.enable();
        }
        drive.arcadeDrive(throttledMove, throttledRotate);
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
        int isDirectionUp;
        // Shooter Lift
        if (joystick.getRawButton(BUTTON_SHOOTER_LIFT_UP)) {
            if ((pot.get()-1950)/17.4 < 420-300) SmartDashboard2.put("targetAngle", ++targetAngle);
            isDirectionUp = 1;
        } else if (joystick.getRawButton(BUTTON_SHOOTER_LIFT_DOWN)) {
            if ((pot.get()-1950)/17.4 < 420-300 && SmartDashboard2.get("targetAngle",0) > 33) {
                SmartDashboard2.put("targetAngle", --targetAngle);
            }
            isDirectionUp = -1;
        }
        else {
            isDirectionUp = 0;
        }
        targetAngle = SmartDashboard2.get("targetAngle", targetAngle);
        double setpoint = targetAngle * 17.4 + 1950;
        potPID.setSetpoint(setpoint);
        SmartDashboard2.put("targetAngle", targetAngle);
        if(!potPID.isEnabled()){
            shooterWheelLift.set(DISABLEDPIDLIFTSPEEDMULTIPLIERCALLMEKYLE * isDirectionUp);
        }
    }

    public void shootAndIntake() {
        double shooterLeftWheelSpeed = SmartDashboard2.get("leftWheelSpeed", SPEED_SHOOTER_WHEEL_LEFT);
        double shooterRightWheelSpeed = SmartDashboard2.get("rightWheelSpeed", SPEED_SHOOTER_WHEEL_RIGHT);
        // Shooter Wheels
        if (joystick.getRawButton(BUTTON_SHOOTER_WHEELSPIN_IN)) {
            if (shooterKickerReturnSensor.get()) shooterKicker.set(SPEED_SHOOTER_KICKER_IN);
            Timer.delay(0.1);
            shooterWheelLeft.set(shooterLeftWheelSpeed);
            shooterWheelRight.set(-shooterRightWheelSpeed);
        } else if (joystick.getRawButton(BUTTON_SHOOTER_WHEELSPIN_OUT)) {
            shooterWheelLeft.set(-shooterLeftWheelSpeed);
            shooterWheelRight.set(shooterRightWheelSpeed);
            Timer.delay(0.5); // Spinning up...
            shooterKicker.set(-SPEED_SHOOTER_KICKER_OUT);
        } else {
            shooterWheelLeft.set(0);
            shooterWheelRight.set(0);
            if (shooterKickerReturnSensor.get()) shooterKicker.set(SPEED_SHOOTER_KICKER_IN);
            else shooterKicker.set(0);
        }
    }

    @Override
    public void disabledInit() {
        System.out.println("Default IterativeRobot.disabledInit() method... Overload me, Chandler!");
    }
}
