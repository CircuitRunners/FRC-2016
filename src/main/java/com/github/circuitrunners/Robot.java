package com.github.circuitrunners;

import com.analog.adis16448.frc.ADIS16448_IMU;
import com.github.circuitrunners.akilib.*;
import com.github.circuitrunners.calib.CalibMath;
import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.buttons.Button;
import edu.wpi.first.wpilibj.buttons.JoystickButton;
import edu.wpi.first.wpilibj.networktables.NetworkTable;
import edu.wpi.first.wpilibj.tables.ITable;
import edu.wpi.first.wpilibj.vision.AxisCamera;

import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;

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

    private static final int PORT_HALL_SENSOR = 0;
    private static final int PORT_JOYSTICK = 0;
    private static final int PORT_XBOX = 1;

    // Drive Adjustments
    private static final double JOYSTICK_DEADZONE = 0.1;
    private static final double JOYSTICK_SCALE_FLAT = 0.7;
    private static final double JOYSTICK_SCALE_POWER = 1;
    private static final double JOYSTICK_DEADZONE_PID = 0.2;
    private static final String DRIVER_CONTROL_TYPE = "Xbox";

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
    private static final double KP_THIS_SHIT = 0;
    private static final double KD_THIS_SHIT = 0;
    private static final double TOLERANCE_THIS_SHIT = 0.1;

    private static final double KP_LIFT = 0;
    private static final double KI_LIFT = 0;
    private static final double KD_LIFT = 0;

    //Shooter Constants
    private static final double SPEED_SHOOTER_WHEEL_LEFT = 1;
    private static final double SPEED_SHOOTER_WHEEL_RIGHT = 1;

    private static final double TOLERANCE_LIFT = 5;
    private static final double ANGLE_LIFT_INCREMENT = 50;
    private static final double SPEED_SHOOTER_KICKER_OUT = 1;
    private static final double SPEED_SHOOTER_KICKER_IN = 0.3;

    private RobotDrive drive;

    private VictorSP shooterWheelLeft;
    private VictorSP shooterWheelRight;

    private CANTalon shooterLift;
    private PIDController shooterLiftPID;
    private DigitalInput liftLimit;

    private CANTalon shooterKicker;

    private Joystick joystick;
    private Xbox xbox;

    // Buttons
    private Button buttonPidEnable;
    private Button buttonGyroReset;

    private Button buttonShooterLiftDown;
    private Button buttonShooterLiftUp;
    private Button resetLift;

    private Button buttonShooterWheelspinOut;
    private Button buttonShooterWheelspinIn;
    private Button buttonShooterKickOut;
    private Button buttonShooterKickIn;

    private ADIS16448_IMU thisShit;
    private PIDController thisPIDController;
    private String weatherStatus;

    private AxisCamera camera;

    private final NetworkTable grip = NetworkTable.getTable("GRIP");

    private ExecutorService sequentialExecutor = Executors.newSingleThreadExecutor();
    private ExecutorService parallelExecutor = Executors.newCachedThreadPool();

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
        shooterLiftPID = new PIDController(KP_LIFT, KI_LIFT, KD_LIFT, shooterLift, shooterLift);
        shooterLiftPID.setAbsoluteTolerance(TOLERANCE_LIFT);
        liftLimit = new DigitalInput(PORT_HALL_SENSOR);

        shooterKicker = new CANTalon(PORT_SHOOTER_KICKER);

        joystick = new Joystick(PORT_JOYSTICK);
        xbox = new Xbox(PORT_XBOX);
        
        thisShit = new ADIS16448_IMU();
        thisShit.calibrate();
        thisPIDController = new PIDController(KP_THIS_SHIT, 0, KD_THIS_SHIT, thisShit, output -> {});
        thisPIDController.setPercentTolerance(TOLERANCE_THIS_SHIT);

        camera = new AxisCamera("10.10.2.11");
    }

    @Override
    public void autonomousInit() {
        sequentialExecutor.execute(new AutonomousDriveThread(1, 0, 1000));
        sequentialExecutor.execute(new AutonomousDriveThread(-1, 0, 1000));
    }

    private class AutonomousDriveThread implements Runnable {
        double moveVal;
        double rotateVal;
        int waitTime;

        public AutonomousDriveThread(double moveVal, double rotateVal) {
            this(moveVal, rotateVal, 0);
        }

        public AutonomousDriveThread(double moveVal, double rotateVal, int waitTime) {
            this.moveVal = moveVal;
            this.rotateVal = rotateVal;
            this.waitTime = waitTime;
        }

        @Override
        public void run() {
            drive.arcadeDrive(moveVal, rotateVal);
            if (waitTime > 0) {
                try {
                    Thread.sleep(waitTime);
                } catch (InterruptedException e) {
                    e.printStackTrace();
                }
            }
        }
    }

    @Override
    public void autonomousPeriodic() {
    }

    @Override
    public void teleopInit() {
        // Just in case...
        thisPIDController.disable();

        while (liftLimit.get()) {
            shooterLift.set(-0.5);
        }
        shooterLift.setEncPosition(0);

        weatherStatus = CalibMath.answerQuestion();

        // Drive Controls
        setControlType(SmartDashboard2.get("driverControlType", DRIVER_CONTROL_TYPE));
    }

    boolean triggerPressed; //so lonely
    @Override
    public void teleopPeriodic() {

        drive();
        liftShooter();
        shootAndIntake();

        // Gyro reset
        if (buttonGyroReset.get()) thisShit.reset();

        // Debug
        // Drive values
        SmartDashboard2.put("derp", CalibMath.adjustedDeadband(joystick.getX(),0.3)); //save "derp" to dictionary

        //Weather
        SmartDashboard2.put("Temperature", thisShit.getTemperature());
        SmartDashboard2.put("Pressure", thisShit.getBarometricPressure());
        SmartDashboard2.put("Will it rain?", weatherStatus);

        SmartDashboard2.put("IMU", thisShit);
       // SmartDashboard2.put("goodToGo", isShooterAimed);

        double[] temp = getStuff();
        if (temp.length == 2) {
            SmartDashboard2.put("distance", temp[0]);
            SmartDashboard2.put("azimuth", temp[1]);
        }
    }

    private void moveStraight(double distance, double speed) {
        // TODO: Test this
        thisPIDController.enable();
        while (thisShit.getMagX() < distance) {
            thisPIDController.setSetpoint(thisShit.getAngle());
            drive.arcadeDrive(speed, thisPIDController.get());
        }
        thisPIDController.disable();
    }

    private void moveTurnways(double angle) {
        //TODO: Test this
        thisPIDController.enable();
        double initialAngle = thisShit.getAngle();
        double finalAngle = initialAngle + angle;
        while (finalAngle < 0 || finalAngle > 360) finalAngle += finalAngle < 0 ? 360 : finalAngle > 360 ? -360 : 0;
        thisPIDController.setSetpoint(finalAngle);
        drive.arcadeDrive(0, thisPIDController.get());
        thisPIDController.disable();
    }

    private void setControlType(String driverControlType) {
        switch (driverControlType) {
            case "JoystickOnly":
                buttonPidEnable = new JoystickButton(joystick, 4);
                buttonGyroReset = new JoystickButton(joystick, 6);

                buttonShooterLiftDown = new JoystickButton(joystick, 3);
                buttonShooterLiftUp = new JoystickButton(joystick, 5);
                resetLift = new JoystickButton(joystick, 9);

                buttonShooterWheelspinOut = new JoystickButton(joystick, 1);
                buttonShooterWheelspinIn = new JoystickButton(joystick, 2);
                buttonShooterKickOut = new Button2(joystick, POVDirection.UP);
                buttonShooterKickIn = new Button2(joystick, POVDirection.DOWN);
                break;
            default:
                buttonPidEnable = new JoystickButton(joystick, 4);
                buttonGyroReset = new JoystickButton(joystick, 6);

                buttonShooterLiftDown = new XboxButton(xbox, Xbox.Button.A);
                buttonShooterLiftUp = new XboxButton(xbox, Xbox.Button.B);
                resetLift = new XboxButton(xbox, Xbox.Button.BACK);

                buttonShooterWheelspinOut = new XboxButton(xbox, Xbox.Button.RIGHT_BUMPER);
                buttonShooterWheelspinIn = new JoystickButton(joystick, 1);
                buttonShooterKickIn = new JoystickButton(xbox, Xbox.Button.X.ordinal());
                buttonShooterKickOut = new JoystickButton(xbox, Xbox.Button.Y.ordinal());
                break;
        }
    }

    private void drive() {
        // Drive values
        double moveVal = SmartDashboard2.put("moveVal", joystick.getRawAxis(AXIS_MOVE));
        double twistVal = SmartDashboard2.put("twistVal", joystick.getRawAxis(AXIS_ROTATE));
        double throttleVal = SmartDashboard2.put("throttleVal",
                                                 CalibMath.throttleMath(-joystick.getRawAxis(AXIS_THROTTLE)));

        double rotateVal = SmartDashboard2.put("rotateVal", CalibMath.scaleDoubleFlat(twistVal,
                                                                                 SmartDashboard2.get("joystickDeadzone", JOYSTICK_DEADZONE),
                                                                                 SmartDashboard2.get("joystickScaleFlat", JOYSTICK_SCALE_FLAT),
                                                                                 SmartDashboard2.get("joystickScalePower", JOYSTICK_SCALE_POWER))); //could make magic numbers into constants but who cares
        double throttledMove = SmartDashboard2.put("throttledMove", (throttleVal) * moveVal); //0% chance we need this elsewhere but who cares
        //there's no code in this line but who cares

        // Gyro values
        double thisRadians = thisShit.getAngle();
        double thisDegrees = Math.toDegrees(thisRadians);

        // Update setpoint if turning
        if (thisPIDController.isEnabled() && Math.abs(rotateVal) > JOYSTICK_DEADZONE_PID) thisPIDController.setSetpoint(thisDegrees);

        // PID values
        SmartDashboard2.put("thisPIDController", thisPIDController);

        // PID Enable/Disable
        if (buttonPidEnable.get() && !triggerPressed) {
            thisPIDController.setSetpoint(thisDegrees);
            triggerPressed = true;
        }
        if (buttonPidEnable.get()) {
            if (thisPIDController.isEnabled()) thisPIDController.enable();
        } else {
            thisPIDController.disable();
            triggerPressed = false;
        }

        double throttledRotate = SmartDashboard2.put("throttledRotate", thisPIDController.get());
        drive.arcadeDrive(throttledMove, throttledRotate);
    }

    public void liftShooter() {
        // Put sensors on SmartDashboard
        SmartDashboard2.put("Hall", liftLimit);

        // Encoder Control
        // Put shooterLift PID control values to SmartDashboard
        SmartDashboard2.put("liftMotor", shooterLift);
        // Create setpoint variable from SmartDashboard value
        double liftSetpoint = SmartDashboard2.get("liftSetpoint", 0);
        // Check button values
        if (buttonShooterLiftUp.get()) {
            SmartDashboard2.put("liftSetpoint", liftSetpoint + ANGLE_LIFT_INCREMENT);
        } else if (buttonShooterLiftDown.get() && liftLimit.get()) {
            SmartDashboard2.put("liftSetpoint", liftSetpoint - ANGLE_LIFT_INCREMENT);
        }
        // Not else if because should reset while holding above buttons
        if (resetLift.get()) SmartDashboard2.put("liftSetpoint", 0);

        // Liftlimit should enable/disable PID
        if (liftLimit.get()) {
            if (!shooterLiftPID.isEnabled()) shooterLiftPID.enable();
        } else {
            shooterLiftPID.disable();
        }
        // Actually adjust the setpoint
        shooterLiftPID.setSetpoint(SmartDashboard2.get("liftSetpoint", 0));
    }

    public void shootAndIntake() {
        double shooterLeftWheelSpeed = SmartDashboard2.get("leftWheelSpeed", SPEED_SHOOTER_WHEEL_LEFT);
        double shooterRightWheelSpeed = SmartDashboard2.get("rightWheelSpeed", SPEED_SHOOTER_WHEEL_RIGHT);
        // Shooter Wheels spin
        if (buttonShooterWheelspinIn.get()) {
            shooterKicker.set(SPEED_SHOOTER_KICKER_IN);
            shooterWheelLeft.set(shooterLeftWheelSpeed);
            shooterWheelRight.set(-shooterRightWheelSpeed);
        } else if (buttonShooterWheelspinOut.get()) {
            shooterWheelLeft.set(-shooterLeftWheelSpeed);
            shooterWheelRight.set(shooterRightWheelSpeed);
            // Inside if to prevent accidental shot
            if (buttonShooterKickOut.get()) shooterKicker.set(-SPEED_SHOOTER_KICKER_OUT);
        } else {
            shooterWheelLeft.set(0);
            shooterWheelRight.set(0);
            shooterKicker.set(0);
        }

        // Kicker wheel spin
        if (buttonShooterKickIn.get()) shooterKicker.set(SPEED_SHOOTER_KICKER_IN);
        else shooterKicker.set(0);
    }

    // Camera stuff bleh
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
