package com.github.circuitrunners;

import com.analog.adis16448.frc.ADIS16448_IMU;
import com.github.circuitrunners.akilib.Button2;
import com.github.circuitrunners.akilib.POVDirection;
import com.github.circuitrunners.akilib.SmartDashboard2;
import com.github.circuitrunners.akilib.Xbox;
import com.github.circuitrunners.calib.CalibMath;
import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.buttons.Button;
import edu.wpi.first.wpilibj.buttons.JoystickButton;
import edu.wpi.first.wpilibj.networktables.NetworkTable;
import edu.wpi.first.wpilibj.tables.ITable;

import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;
import java.util.concurrent.ScheduledExecutorService;

public class Robot extends IterativeRobot {

    //Port Constants
    public static final int PORT_DRIVE_FRONT_LEFT = 0;
    public static final int PORT_DRIVE_REAR_LEFT = 1;
    public static final int PORT_DRIVE_FRONT_RIGHT = 2;
    public static final int PORT_DRIVE_REAR_RIGHT = 3;
    
    private static final int PORT_SHOOTER_LEFT = 4;
    private static final int PORT_SHOOTER_RIGHT = 5;
    private static final int PORT_SHOOTER_LIFT = 1;
    private static final int PORT_SHOOTER_KICKER = 2;

    private static final int PORT_HALL_SENSOR = 0;
    private static final int PORT_JOYSTICK = 0;
    private static final int PORT_XBOX = 1;

    // Drive Adjustments
    private static final double JOYSTICK_DEADZONE = 0.1;
    private static final double JOYSTICK_SCALE_FLAT = 1;
    private static final double JOYSTICK_SCALE_POWER = 1;
    private static final double JOYSTICK_DEADZONE_PID = 0.2;
    private static final int DRIVER_CONTROL_TYPE = 1;

    // Camera Constants
    private static final double CAMERA_RESOLUTION_Y = 480;
    private static final double CAMERA_RESOLUTION_X = 640;
    private static final double TOP_TARGET_HEIGHT = 97;
    private static final double VERTICAL_FOV = 51.4;
    private static final double HORIZONTAL_FOV = 67;
    private static final double CAMERA_ANGLE = 13;
    private static final double TOP_CAMERA_HEIGHT = 14;

    // Axes
    private static final int AXIS_MOVE = 1;
    private static final int AXIS_ROTATE = 2;
    private static final int AXIS_THROTTLE = 3;

    //PID Constants
    private static final double KP_THIS_SHIT = 0;
    private static final double KD_THIS_SHIT = 0;
    private static final double TOLERANCE_THIS_SHIT = 0.1;

    private static final double KP_LIFT = -0.001;
    private static final double KI_LIFT = 0;
    private static final double KD_LIFT = 0;

    //Shooter Constants
    private static final double SPEED_SHOOTER_WHEEL_LEFT = 1;
    private static final double SPEED_SHOOTER_WHEEL_RIGHT = 1;

    private static final double TOLERANCE_LIFT = 5;
    private static final double ANGLE_LIFT_INCREMENT = 90;
    private static final double SPEED_SHOOTER_KICKER_OUT = 1;
    private static final double SPEED_SHOOTER_KICKER_IN = 1;

    public RobotDrive drive;

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
    private Button buttonDisableLimit;
    private Button buttonEnableLimit;
    private Button resetLift;

    private Button buttonShooterOut;
    //private Button buttonShooterWheelspinIn;
    private Button buttonShooterOutSlow;
    private Button buttonShooterIn;

    private ADIS16448_IMU thisShit;
    private PIDController thisPIDController;
    private final NetworkTable grip = NetworkTable.getTable("GRIP");

    private Encoder encoder;

    private final ExecutorService sequentialExecutor = Executors.newSingleThreadExecutor();
    private final ExecutorService parallelExecutor = Executors.newCachedThreadPool();
    private final ScheduledExecutorService repeatedExecutor = Executors.newSingleThreadScheduledExecutor();

    private Runnable shootOut = new ShooterOutSet(SPEED_SHOOTER_WHEEL_LEFT);
    private Runnable shootOutSlow = new ShooterOutSet(SPEED_SHOOTER_WHEEL_LEFT * 0.5);
    private Runnable shootIn = new ShooterInSet();

    @Override
    public void robotInit() {
        SmartDashboard2.setNetwork(true);
        VictorSP frontLeft = new VictorSP(PORT_DRIVE_FRONT_LEFT);
        VictorSP rearLeft = new VictorSP(PORT_DRIVE_REAR_LEFT);
        VictorSP frontRight = new VictorSP(PORT_DRIVE_FRONT_RIGHT);
        VictorSP rearRight = new VictorSP(PORT_DRIVE_REAR_RIGHT);

        drive = new RobotDrive(frontLeft, rearLeft, frontRight, rearRight);

        drive.setExpiration(5);

        shooterWheelLeft = new VictorSP(PORT_SHOOTER_LEFT);
        shooterWheelRight = new VictorSP(PORT_SHOOTER_RIGHT);

        shooterLift = new CANTalon(PORT_SHOOTER_LIFT);
        CANTalon localShooterLift = shooterLift;
        shooterLiftPID = new PIDController(KP_LIFT, KI_LIFT, KD_LIFT, localShooterLift, localShooterLift);
        shooterLiftPID.setAbsoluteTolerance(TOLERANCE_LIFT);
        liftLimit = new DigitalInput(PORT_HALL_SENSOR);

        shooterKicker = new CANTalon(PORT_SHOOTER_KICKER);

        joystick = new Joystick(PORT_JOYSTICK);
        xbox = new Xbox(PORT_XBOX);
        
        thisShit = new ADIS16448_IMU();
        ADIS16448_IMU localIMU = thisShit;
        localIMU.calibrate();
        thisPIDController = new PIDController(KP_THIS_SHIT, 0, KD_THIS_SHIT, localIMU, output -> {});
        thisPIDController.setPercentTolerance(TOLERANCE_THIS_SHIT);

        encoder = new Encoder(3 ,4);
        encoder.reset();

        /*AxisCamera camera = new AxisCamera("10.10.2.11");
        try {
            camera.getImage();
        } catch (NIVisionException e) {
            e.printStackTrace();
        } */
    }

    private final DigitalInput directionSwitch = new DigitalInput(1);
    private final DigitalInput timeoutSwitch = new DigitalInput(2);

    private class HomeThread implements Runnable {
        int timeout;

        private HomeThread(int homeTimeout){
            timeout = homeTimeout;
        }

        @Override
        public void run() {
            long last = System.currentTimeMillis();
            long curr;
            long diff = 0;
            while (diff < timeout && liftLimit.get()) {
                shooterLift.set(0.75);
                curr = System.currentTimeMillis();
                diff = curr - last;
            }
            shooterLift.set(0);
            shooterLift.setEncPosition(0);
        }

    }

    private class AutonomousDriveThread implements Runnable {
        double moveVal;
        double rotateVal;
        int delay;
        int waitTime;

        private AutonomousDriveThread(double moveVal, double rotateVal) {
            this(moveVal, rotateVal, 0);
        }

        private AutonomousDriveThread(double moveVal, double rotateVal, int waitTime) {
            this.moveVal = moveVal;
            this.rotateVal = rotateVal;
            this.waitTime = waitTime;
        }

        private AutonomousDriveThread(double moveVal, double rotateVal, int delay, int waitTime) {
            this.moveVal = moveVal;
            this.rotateVal = rotateVal;
            this.delay = delay;
            this.waitTime = waitTime;
        }

        @Override
        public void run() {
            if (delay > 0) {
                try {
                    Thread.sleep(delay);
                } catch (InterruptedException e) {
                    e.printStackTrace();
                }
            }
            drive.arcadeDrive(moveVal, rotateVal);
            if (waitTime > 0) {
                try {
                    Thread.sleep(waitTime);
                    drive.arcadeDrive(0  ,  0); //hoot hoot bitch
                } catch (InterruptedException e) {
                    e.printStackTrace();
                }
            }
        }
    }

    @Override
    public void autonomousInit() {
        sequentialExecutor.execute(new HomeThread(timeoutSwitch.get() ? 3000 : 0));
        shooterLiftPID.disable();
        // TODO: Add switch for direction
        sequentialExecutor.execute(new AutonomousDriveThread(directionSwitch.get() ? 0.8 : -0.8, 0, 0, 5000));
        System.out.println("timeout " + timeoutSwitch.get());
        System.out.println("direction " +  directionSwitch.get());
    }

    @Override
    public void autonomousPeriodic() {
    }


    @Override
    public void teleopInit() {
        // Just in case...
        thisPIDController.disable();

       // sequentialExecutor.execute(new HomeThread(timeoutSwitch.get() ? 3000 : 0));
        shooterLiftPID.disable();
        shooterLiftPID.setSetpoint(SmartDashboard2.put("liftSetpoint",0));

        // Drive Controls
        setControlType(SmartDashboard2.get("driverControlType", DRIVER_CONTROL_TYPE));
    }

    boolean triggerPressed; //so lonely
    boolean liftSwitchEnabled = true;

    @Override
    public void teleopPeriodic() {

        drive();
        liftShooter();
        shootAndIntake();
        if(joystick.getRawButton(9)){
            encoder.reset();
            while(encoder.getDistance() > -430){
                drive.arcadeDrive(-.8,0);
            }
        }

        // Gyro reset
        if (buttonGyroReset.get()) {
            thisShit.reset();
        }

        // Debug
        // Drive values
        SmartDashboard2.put("Deadband", CalibMath.adjustedDeadband(joystick.getX(),0.3)); //save "derp" to dictionary

        SmartDashboard2.put("IMU", thisShit);
       // SmartDashboard2.put("goodToGo", isShooterAimed);
        SmartDashboard2.put("encoder", encoder.getDistance());

       /* double[] temp = getStuff();
        if (temp.length == 2) {
            SmartDashboard2.put("distance", temp[0]);
            SmartDashboard2.put("azimuth", temp[1]);
        }*/
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
        while (finalAngle < 0 || finalAngle > 360) {
            finalAngle += finalAngle < 0 ? 360 : finalAngle > 360 ? -360 : 0;
        }
        thisPIDController.setSetpoint(finalAngle);
        drive.arcadeDrive(0, thisPIDController.get());
        thisPIDController.disable();
    }

    private void setControlType(int driverControlType) {
        if (driverControlType == 0) {
            buttonPidEnable = new JoystickButton(joystick, 4);
            buttonGyroReset = new JoystickButton(joystick, 6);

            buttonShooterLiftDown = new JoystickButton(joystick, 3);
            buttonShooterLiftUp = new JoystickButton(joystick, 5);
            resetLift = new JoystickButton(joystick, 9);

            buttonShooterOut = new JoystickButton(joystick, 1);
            //buttonShooterWheelspinIn = new JoystickButton(joystick, 2);
            buttonShooterOutSlow = new Button2(joystick, POVDirection.UP);
            buttonShooterIn = new Button2(joystick, POVDirection.DOWN);
        } else {
            buttonPidEnable = new JoystickButton(joystick, 4);
            buttonGyroReset = new JoystickButton(joystick, 6);

            buttonShooterLiftDown = new Button2(xbox, POVDirection.DOWN);
            buttonShooterLiftUp = new Button2(xbox, POVDirection.UP);
            buttonDisableLimit = new JoystickButton(xbox, 7);
            buttonEnableLimit = new JoystickButton(xbox, 8);
            resetLift = new JoystickButton(xbox, 1);

            buttonShooterOut = new JoystickButton(xbox, 5);
            //buttonShooterWheelspinIn = new JoystickButton(joystick, 2);
            buttonShooterIn = new JoystickButton(joystick, 1);
            buttonShooterOutSlow = new JoystickButton(xbox, 6);
        }
    }

    private void drive() {
        // Drive values
        double moveVal = SmartDashboard2.put("moveVal", joystick.getY());
        double twistVal = SmartDashboard2.put("twistVal", joystick.getTwist());
        double throttleVal = SmartDashboard2.put("throttleVal",
                                                 CalibMath.throttleMath(joystick.getThrottle()));

        double rotateVal = SmartDashboard2.put("rotateVal", CalibMath.scaleDoubleFlat(twistVal,
                                                                                 SmartDashboard2.get("joystickDeadzone", JOYSTICK_DEADZONE),
                                                                                 SmartDashboard2.get("joystickScaleFlat", JOYSTICK_SCALE_FLAT),
                                                                                 SmartDashboard2.get("joystickScalePower", JOYSTICK_SCALE_POWER))); //could make magic numbers into constants but who cares
        double throttledMove = SmartDashboard2.put("throttledMove", throttleVal * moveVal); //0% chance we need this elsewhere but who cares

        // Gyro values
        double thisRadians = thisShit.getAngle();
        double thisDegrees = Math.toDegrees(thisRadians);

        // Update setpoint if turning
        if (thisPIDController.isEnabled() && Math.abs(rotateVal) > JOYSTICK_DEADZONE_PID) {
            thisPIDController.setSetpoint(thisDegrees);
        }

        // PID values
        SmartDashboard2.put("thisPIDController", thisPIDController);

        // PID Enable/Disable
        if (buttonPidEnable.get() && !triggerPressed) {
            thisPIDController.setSetpoint(thisDegrees);
            triggerPressed = true;
        }
        if (buttonPidEnable.get()) {
            if (thisPIDController.isEnabled()) {
                thisPIDController.enable();
            }
        } else {
            thisPIDController.disable();
            triggerPressed = false;
        }

        double throttledRotate = SmartDashboard2.put("throttledRotate", CalibMath.inverseAdjustedDeadband(throttleVal, 0.5) * rotateVal
                                                    + thisPIDController.get());
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
        if(buttonEnableLimit.get()) {
            liftSwitchEnabled = true;
        }
        if(buttonDisableLimit.get()) {
            liftSwitchEnabled = false;
        }
        if(liftSwitchEnabled) { //If limit switch doesnt break
            if (buttonShooterLiftUp.get() && shooterLiftPID.isEnabled() /*&& SmartDashboard2.get("liftSetpoint", 0) + liftSetpoint + ANGLE_LIFT_INCREMENT < 600*/) {
                SmartDashboard2.put("liftSetpoint", liftSetpoint + ANGLE_LIFT_INCREMENT);
            } else if (buttonShooterLiftDown.get() && liftLimit.get() && shooterLiftPID.isEnabled()) {
                SmartDashboard2.put("liftSetpoint", liftSetpoint - ANGLE_LIFT_INCREMENT);
            }
            // Not else if because should reset while holding above buttons
            if (resetLift.get()) {
                SmartDashboard2.put("liftSetpoint", 0);
            }

            // Liftlimit should enable/disable PID
            if (liftLimit.get()) {
                if (!shooterLiftPID.isEnabled()) {
                    shooterLiftPID.enable();
                }
            } else {
                shooterLiftPID.disable();
                if (buttonShooterLiftUp.get()) {
                    shooterLift.set(-0.5);
                } else if (buttonShooterLiftDown.get()) {
                    shooterLift.set(0.5);
                } else {
                    shooterLift.set(0);
                }
                return;
            }
            // Actually adjust the setpoint
            shooterLiftPID.setSetpoint(SmartDashboard2.get("liftSetpoint", 0));
        }
        else{
            shooterLiftPID.disable();
            if (buttonShooterLiftUp.get()) {
                shooterLift.set(-0.5);
            } else if (buttonShooterLiftDown.get()) {
                shooterLift.set(0.5);
            } else {
                shooterLift.set(0);
            }
        }
        SmartDashboard2.put("shooterLiftPosition", shooterLift.getEncPosition());
        SmartDashboard2.put("shooterLiftPID", shooterLiftPID);
    }

    public void shootAndIntake() {
        if (buttonShooterOut.get() || buttonShooterOutSlow.get() || buttonShooterIn.get()) {
            if (buttonShooterOut.get()) {
                parallelExecutor.execute(shootOut);
            } else if (buttonShooterOutSlow.get()) {
                parallelExecutor.execute(shootOutSlow);
            } else if (buttonShooterIn.get()) {
                parallelExecutor.execute(shootIn);
            }
        } else {
            shooterWheelLeft.set(0);
            shooterWheelRight.set(0);
            shooterKicker.set(0);
        }
    }

    public class ShooterOutSet implements Runnable {

        private double speed;

        public ShooterOutSet(double speed) {
            this.speed = speed;
        }

        @Override
        public void run() {
            shooterWheelLeft.set(-speed);
            shooterWheelRight.set(speed);
            try {
                Thread.sleep(500);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
            shooterKicker.set(-SPEED_SHOOTER_KICKER_OUT);
        }
    }

    public class ShooterInSet implements Runnable {

        public ShooterInSet() {

        }

        @Override
        public void run() {
            shooterKicker.set(SPEED_SHOOTER_KICKER_IN);
            try {
                Thread.sleep(200);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
            shooterWheelLeft.set(1);
            shooterWheelRight.set(-1);
        }
    }

    // Camera stuff bleh
    public double[] getStuff(){
        ITable table = grip.getSubTable("contours");

        double[] emptyArr = new double[0];

        int largest = CalibMath.getLargestIndex(table.getNumberArray("area", emptyArr));

        double[] centerXs = table.getNumberArray("centerX", emptyArr);
        double centerX = 0.0;
        if (largest < centerXs.length) {
            centerX = centerXs[largest];
        }

        double[] centerYs = table.getNumberArray("centerY", emptyArr);
        double centerY = 0.0;
        if (largest < centerXs.length) {
            centerY = centerYs[largest];
        }

        // double[] widths = table.getNumberArray("width", new double[0]);
        // double width = widths[largest]; TODO: use these

        // double[] heights = table.getNumberArray("height", new double[0]);
        // double height = heights[largest]; TODO: use these

        // double[] areas = table.getNumberArray("area", new double[0]);
        // double area = areas[largest]; TODO: use these

        double y = -(2 * centerY / CAMERA_RESOLUTION_Y - 1);
        double distance = (TOP_TARGET_HEIGHT - TOP_CAMERA_HEIGHT) /
                Math.tan((y * VERTICAL_FOV / 2.0 + CAMERA_ANGLE) * Math.PI / 180);
        //				angle to target...would not rely on this
        double targetX = 2 * centerX / CAMERA_RESOLUTION_X - 1;
        double azimuth = CalibMath.normalize360(targetX * HORIZONTAL_FOV / 2.0 + 0);

        return new double[]{distance, azimuth};
    }

    @Override
    public void disabledInit() {
    }
}
