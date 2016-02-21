
package com.github.circuitrunners;

import com.analog.adis16448.frc.ADIS16448_IMU;
import com.github.circuitrunners.akilib.PIDSource2;
import com.github.circuitrunners.akilib.SmartDashboard2;
import com.github.circuitrunners.akilib.XboxButton;
import com.github.circuitrunners.calib.CalibMath;
import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.buttons.JoystickButton;
import edu.wpi.first.wpilibj.networktables.NetworkTable;
import edu.wpi.first.wpilibj.tables.ITable;
import edu.wpi.first.wpilibj.vision.AxisCamera;

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
    private static final int PORT_JOYSTICK = 0;
    private static final int PORT_XBOX = 1;

    // Drive Adjustments
    private static final double JOYSTICK_DEADZONE = 0.1;
    private static final double JOYSTICK_SCALE_FLAT = 0.7;
    private static final double JOYSTICK_SCALE_POWER = 1;
    private static final double JOYSTICK_DEADZONE_PID = 0.2;
    private static final double DISABLEDPIDLIFTSPEEDMULTIPLIERCALLMEKYLE = 0.3;
    private static final String DRIVER_CONTROL_TYPE = "VERSION_1";

    // Camera Constants
    private static final double CAMERA_RESOLUTION_Y = 480;
    private static final double CAMERA_RESOLUTION_X = 640;
    private static final double TOP_TARGET_HEIGHT = 97;
    private static final double VERTICAL_FOV = 51.4;
    private static final double HORIZONTAL_FOV = 67;
    private static final double CAMERA_ANGLE = 13;
    private static final double TOP_CAMERA_HEIGHT = 14;


    // Axes
    private static int AXIS_MOVE = 1;
    private static int AXIS_ROTATE = 2;
    private static int AXIS_THROTTLE = 3;

    //PID Constants
    private static final double KP_POT = 0.002;
    private static final double KI_POT = 0.001;
    private static final double KD_POT = 0.001;

    private static final double KP_THIS_SHIT = 0;
    private static final double KD_THIS_SHIT = 0;
    private static final double TOLERANCE_THIS_SHIT = 0.1;

    private static final double KP_THAT_SHIT = 0.1;
    private static final double KD_THAT_SHIT = 0.1;

    public static final double KP_THE_OTHER_SHIT = 0.05;
    public static final double KD_THE_OTHER_SHIT = 0.1;

    //Shooter Constants
    private static final double SPEED_SHOOTER_WHEEL_LEFT = 1;
    private static final double SPEED_SHOOTER_WHEEL_RIGHT = 1;

    private static final double ANGLE_LIFT_REST = 28;
    private static final double TOLERANCE_PID_POT = 0.1;
//    private static final double OFFSET_SHOOTER = 0.17;
    private static final double SPEED_SHOOTER_LIFT_UP = 0.5;
    private static final double SPEED_SHOOTER_LIFT_DOWN = 0.5;
    private static final double SPEED_SHOOTER_KICKER_OUT = 1;
    private static final double SPEED_SHOOTER_KICKER_IN = 0.3;


    private RobotDrive drive;

    private VictorSP shooterWheelLeft;
    private VictorSP shooterWheelRight;

    private CANTalon shooterLift;

    private CANTalon shooterKicker;
    private DigitalInput liftLimit;

    private Joystick joystick;
    private Joystick xbox;

    // Buttons
    private JoystickButton buttonPidEnable;
    private JoystickButton buttonGyroReset;

    private JoystickButton buttonShooterLiftDown;
    private JoystickButton buttonShooterLiftUp;
    private JoystickButton resetLift;

    private JoystickButton buttonShooterWheelspinOut;
    private JoystickButton buttonShooterWheelspinIn;

    private AnalogGyro theOtherShit;
    private PIDController theOtherPIDController;

    private ADIS16448_IMU thisShit;
    private PIDController thisPIDController;
    
    private ADXRS450_Gyro thatShit;
    private PIDController thatPIDController;
    private String weatherStatus;

    private AnalogPotentiometer pot;

    //private double thisAdjustment;
    private double thatAdjustment;
    private double theOtherAdjustment;
    private PIDController potPID;
    private double targetAngle;

    private AxisCamera camera;

    private final NetworkTable grip = NetworkTable.getTable("GRIP");

    @Override
    public void robotInit() {

        VictorSP frontLeft = new VictorSP(PORT_DRIVE_FRONT_LEFT);
        VictorSP rearLeft = new VictorSP(PORT_DRIVE_REAR_LEFT);
        VictorSP frontRight = new VictorSP(PORT_DRIVE_FRONT_RIGHT);
        VictorSP rearRight = new VictorSP(PORT_DRIVE_REAR_RIGHT);

        drive = new RobotDrive(frontLeft, rearLeft, frontRight, rearRight);

        drive.setExpiration(5);

        shooterWheelLeft = new VictorSP(PORT_SHOOTER_LEFT);
        shooterWheelRight = new VictorSP(PORT_SHOOTER_RIGHT);
        shooterLift = new CANTalon(PORT_SHOOTER_LIFT);
        shooterKicker = new CANTalon(PORT_SHOOTER_KICKER);
        liftLimit = new DigitalInput(0);

        pot = new AnalogPotentiometer(1,5000); // Range: 1950-4560
        potPID = new PIDController(KP_POT, KI_POT, KD_POT, pot, shooterLift);
        potPID.setPercentTolerance(TOLERANCE_PID_POT);
        potPID.enable();

        joystick = new Joystick(PORT_JOYSTICK);
        xbox = new Joystick(PORT_XBOX);
        
        thisShit = new ADIS16448_IMU();
        thisShit.calibrate();
        thisPIDController = new PIDController(KP_THIS_SHIT, 0, KD_THIS_SHIT, thisShit, output -> {});
        thisPIDController.setPercentTolerance(TOLERANCE_THIS_SHIT);

        thatShit = new ADXRS450_Gyro();
        thatShit.calibrate();
        thatPIDController = new PIDController(KP_THAT_SHIT, 0, KD_THAT_SHIT, thatShit, output -> {});

        theOtherShit = new AnalogGyro(0);
        theOtherShit.calibrate();
        theOtherPIDController = new PIDController(KP_THE_OTHER_SHIT, 0, KD_THE_OTHER_SHIT, theOtherShit, output -> {});

        camera = new AxisCamera("10.10.2.11");
    }

    @Override
    public void autonomousInit() {

    }

    @Override
    public void autonomousPeriodic() {

    }

    @Override
    public void teleopInit() {

        // Just in case...
        thisPIDController.disable();
        thatPIDController.disable();
        theOtherPIDController.disable();

        weatherStatus = CalibMath.answerQuestion();

        // Drive Controls
        setControlType(SmartDashboard2.get("driverControlType", DRIVER_CONTROL_TYPE));

        targetAngle = SmartDashboard2.put("targetAngle", ANGLE_LIFT_REST);
    }

    boolean triggerPressed; //so lonely

    @Override
    public void teleopPeriodic() {

        drive();
        liftShooter();
        shootAndIntake();

        // Gyro reset
        if (buttonGyroReset.get()) {
            thisShit.reset();
            thatShit.reset();
            theOtherShit.reset();
        }

        // Debug
        // Drive values
        SmartDashboard2.put("derp", CalibMath.adjustedDeadband(joystick.getX(),0.3)); //save "derp" to dictionary
        SmartDashboard2.put("angle", (pot.get() - 1950) / 17.4);

        //Weather
        SmartDashboard2.put("Temperature", thisShit.getTemperature());
        SmartDashboard2.put("Pressure", thisShit.getBarometricPressure());
        SmartDashboard2.put("Will it rain?", weatherStatus);

       // SmartDashboard2.put("goodToGo", isShooterAimed);
        SmartDashboard2.put("distanceTraveledX", thisShit.getMagX());
        SmartDashboard2.put("distanceTraveledY", thisShit.getMagY());
        SmartDashboard2.put("distanceTraveledZ", thisShit.getMagZ());

        double[] temp = getStuff();
        if (temp != null) {
            SmartDashboard2.put("distance", temp[0]);
            SmartDashboard2.put("azimuth", temp[1]);
        }
    }

    private void moveDistance(double distance) {
        double kP = SmartDashboard2.put("movement_kP", 0);
        double kI = SmartDashboard2.put("movement_kI", 0);
        double kD = SmartDashboard2.put("movement_kD", 0);
        // TODO: PID Stuff
        PIDSource2 acceleromter = new PIDSource2(thisShit.getMagX());
        PIDController accelPID = new PIDController(kP, kI, kD, acceleromter, output -> {});
        drive.drive(accelPID.get(), 0);
    }

    private void setControlType(String driverControlType) {
        switch (driverControlType) {
            case "VERSION_1":
                buttonPidEnable = new JoystickButton(joystick, 4);
                buttonGyroReset = new JoystickButton(joystick, 6);

                buttonShooterLiftDown = new JoystickButton(joystick, 3);
                buttonShooterLiftUp = new JoystickButton(joystick, 5);
                resetLift = new JoystickButton(joystick, 9);

                buttonShooterWheelspinOut = new JoystickButton(joystick, 1);
                buttonShooterWheelspinIn = new JoystickButton(joystick, 2);
                break;
            case "VERSION_2":
                buttonPidEnable = new JoystickButton(joystick, 4);
                buttonGyroReset = new JoystickButton(joystick, 6);

                buttonShooterLiftDown = new JoystickButton(xbox, XboxButton.A.ordinal());
                buttonShooterLiftUp = new JoystickButton(joystick, XboxButton.B.ordinal());
                resetLift = new JoystickButton(joystick, XboxButton.BACK.ordinal());

                buttonShooterWheelspinOut = new JoystickButton(joystick, XboxButton.RIGHT_BUMPER.ordinal());
                buttonShooterWheelspinIn = new JoystickButton(joystick, XboxButton.LEFT_BUMPER.ordinal());
                break;
            case "VERSION_3":
                break;
        }
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
        double thisRadians = thisShit.getAngle();
        double thisDegrees = Math.toDegrees(thisRadians);
        double thatDegrees = SmartDashboard2.put("thatDegrees", thatShit.getAngle());
        double theOtherDegrees = SmartDashboard2.put("theOtherDegrees", theOtherShit.getAngle());

        //Smart Gyro™
        SmartDashboard2.put("SmartGyro", CalibMath.average(CalibMath.normalize360(thatDegrees),
                                                           CalibMath.normalize360(theOtherDegrees)));

        double throttledRotate = SmartDashboard2.put("throttledRotate",
                                                     CalibMath.inverseAdjustedDeadband(throttleVal,0.5) * rotateVal
                                                     + pidAdjustDrive(thisPIDController, thisDegrees, rotateVal));
        drive.arcadeDrive(throttledMove, throttledRotate);
    }

    private double pidAdjustDrive(PIDController pidController, double setpoint, double rotateVal) {
        // PID values
        SmartDashboard2.put("thisPIDController", thisPIDController);
        SmartDashboard2.put("thatPIDController", thatPIDController);
        SmartDashboard2.put("theOtherPIDController", theOtherPIDController);

        // PID Enable/Disable
        if (buttonPidEnable.get() && !triggerPressed) {
            pidController.setSetpoint(setpoint);
            triggerPressed = true;
        }
        if (buttonPidEnable.get()){
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
        // Direction that lift should travel
        int liftDirection = 0;

        // Put sensors on SmartDashboard
        SmartDashboard2.put("Hall", liftLimit);
        SmartDashboard2.put("pot", pot);
        SmartDashboard2.put("PotPID", potPID);

        // Set target angle to SmartDashboard
        if (buttonShooterLiftUp.get()) {
            if ((pot.get()-1950)/17.4 < 420-300) {
                targetAngle += SPEED_SHOOTER_LIFT_UP;
                SmartDashboard2.put("targetAngle", targetAngle);
            }
        } else if (buttonShooterLiftDown.get()) {
            if ((pot.get()-1950)/17.4 < 420-300 && liftLimit.get()) {
                targetAngle -= SPEED_SHOOTER_LIFT_DOWN;
                SmartDashboard2.put("targetAngle", targetAngle);
            }
        } else if(resetLift.get()) SmartDashboard2.put("targetAngle", ANGLE_LIFT_REST);

        // Get angle from SmartDashboard and set setpoint
        targetAngle = SmartDashboard2.get("targetAngle", targetAngle);
        double setpoint = targetAngle * 17.4 + 1950;
        potPID.setSetpoint(setpoint);


        if (Math.abs(SmartDashboard2.get("throttledMove", 0)) > 0.2 || SmartDashboard2.get("targetAngle", 110) > 110
            || !liftLimit.get()) {
            potPID.disable();
        } else {
            potPID.enable();
        }

        // Set lift direction
        if (buttonShooterLiftUp.get()) liftDirection = 1;
        else if (buttonShooterLiftDown.get() && liftLimit.get()) liftDirection = -1;

        if(!potPID.isEnabled()){
            shooterLift.set(DISABLEDPIDLIFTSPEEDMULTIPLIERCALLMEKYLE * liftDirection);
        }
    }

    private Timer shootTimer = new Timer();
    public void shootAndIntake() {
        double shooterLeftWheelSpeed = SmartDashboard2.get("leftWheelSpeed", SPEED_SHOOTER_WHEEL_LEFT);
        double shooterRightWheelSpeed = SmartDashboard2.get("rightWheelSpeed", SPEED_SHOOTER_WHEEL_RIGHT);
        // Shooter Wheels
        if (buttonShooterWheelspinIn.get()) {
            shooterKicker.set(SPEED_SHOOTER_KICKER_IN);
            shooterWheelLeft.set(shooterLeftWheelSpeed);
            shooterWheelRight.set(-shooterRightWheelSpeed);
        } else if (buttonShooterWheelspinOut.get()) {
            shooterWheelLeft.set(-shooterLeftWheelSpeed);
            shooterWheelRight.set(shooterRightWheelSpeed);
            shootTimer.start();
            if (shootTimer.hasPeriodPassed(1)) {
                shooterKicker.set(-SPEED_SHOOTER_KICKER_OUT);
                shootTimer.stop();
                shootTimer.reset();
            }
        } else {
            shooterWheelLeft.set(0);
            shooterWheelRight.set(0);
            shooterKicker.set(0);
        }
    }

    public double[] getStuff(){
        try {
            final ITable table = grip.getSubTable("contours");

            int largest = CalibMath.getLargestIndex(table.getNumberArray("area", new double[0]));

            double[] centerXs = table.getNumberArray("centerX", new double[0]);
            double centerX = centerXs[largest];

            double[] centerYs = table.getNumberArray("centerY", new double[0]);
            double centerY = centerYs[largest];

            double[] widths = table.getNumberArray("width", new double[0]);
            double width = widths[largest];

            double[] heights = table.getNumberArray("height", new double[0]);
            double height = heights[largest];

            double[] areas = table.getNumberArray("area", new double[0]);
            double area = areas[largest];

            double y = -((2 * (centerY / CAMERA_RESOLUTION_Y)) - 1);
            double distance = (TOP_TARGET_HEIGHT - TOP_CAMERA_HEIGHT) /
                    Math.tan((y * VERTICAL_FOV / 2.0 + CAMERA_ANGLE) * Math.PI / 180);
            //				angle to target...would not rely on this
            double targetX = (2 * (centerX / CAMERA_RESOLUTION_X)) - 1;
            double azimuth = CalibMath.normalize360(targetX * HORIZONTAL_FOV / 2.0 + 0);

            double[] values = {distance, azimuth};
            return values;
        } catch (ArrayIndexOutOfBoundsException e) {
            return new double[]{0, 0};
        }
    }

    @Override
    public void disabledInit() {
        System.out.println("Not default IterativeRobot.disabledInit() method... Overload me, Chandler!");
    }
}
