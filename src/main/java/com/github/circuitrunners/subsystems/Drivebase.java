package com.github.circuitrunners.subsystems;

import com.github.circuitrunners.RobotMap;
import com.github.circuitrunners.akilib.PIDSource2;
import com.github.circuitrunners.akilib.SmartDashboard2;
import com.github.circuitrunners.calib.CalibMath;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.command.Subsystem;

/**
 * Created by Akilan on 12.03.2016.
 */
public class Drivebase extends Subsystem {

    private static final double KP = 0;
    private static final double KI = 0;
    private static final double KD = 0;
    // Axes
    public static int AXIS_MOVE = 1;
    public static int AXIS_ROTATE = 2;
    public static int AXIS_THROTTLE = 3;

    // Drive Adjustments
    private static final double JOYSTICK_DEADZONE = 0.1;
    private static final double JOYSTICK_SCALE_FLAT = 0.7;
    private static final double JOYSTICK_SCALE_POWER = 1;


    public Drivebase() {
    }

    public void moveDistance(double distance) {
        // TODO: PID Stuff
        PIDSource2 acceleromter = new PIDSource2(RobotMap.imu.getMagX());
        PIDController accelPID = new PIDController(KP, KI, KD, acceleromter, output -> {});
        RobotMap.robotDrive.drive(accelPID.get(), 0);
    }

    public void arcadeDrive(double moveVal, double twistVal) {
        double throttleVal = SmartDashboard2.put("throttleVal",
                                                 CalibMath.throttleMath(-RobotMap.joystick.getThrottle()));
        double throttledMove = throttleVal * SmartDashboard2.put("moveVal", moveVal);
        double throttleRotate = throttleVal *
                                SmartDashboard2.put("rotateVal",
                                                      CalibMath.scaleDoubleFlat(twistVal,
                                                            SmartDashboard2.get("joystickDeadzone",
                                                                                JOYSTICK_DEADZONE),
                                                            SmartDashboard2.get("joystickScaleFlat",
                                                                                JOYSTICK_SCALE_FLAT),
                                                            SmartDashboard2.get("joystickScalePower",
                                                                                JOYSTICK_SCALE_POWER)));
        RobotMap.robotDrive.arcadeDrive(throttledMove, throttleRotate);
    }

    public void arcadeDrive() {
        // Drive values
        double moveVal = SmartDashboard2.put("moveVal", -RobotMap.joystick.getY());
        double twistVal = SmartDashboard2.put("twistVal", RobotMap.joystick.getTwist());
        arcadeDrive(moveVal, twistVal);
    }

    @Override
    public void initDefaultCommand() {
    }
}