package com.github.circuitrunners;

public class RobotValueMap {
    //Port Constants
    public static final int PORT_DRIVE_FRONT_LEFT = 0;
    public static final int PORT_DRIVE_REAR_LEFT = 1;
    public static final int PORT_DRIVE_FRONT_RIGHT = 2;
    public static final int PORT_DRIVE_REAR_RIGHT = 3;

    public static final int PORT_SHOOTER_LEFT = 4;
    public static final int PORT_SHOOTER_RIGHT = 5;
    public static final int PORT_SHOOTER_KICKER = 2;

    public static final int PORT_LIFT = 1;

    public static final int PORT_SENSOR_LIFT = 0;
    public static final int PORT_JOYSTICK = 0;
    public static final int PORT_XBOX = 1;

    // Drive Adjustments
    public static final double JOYSTICK_DEADZONE = 0.1;
    public static final double JOYSTICK_SCALE_FLAT = 1;
    public static final double JOYSTICK_SCALE_POWER = 1;
    public static final double JOYSTICK_DEADZONE_PID = 0.2;
    public static final int DRIVER_CONTROL_TYPE = 1;

    // Camera Constants
    public static final double CAMERA_RESOLUTION_Y = 480;
    public static final double CAMERA_RESOLUTION_X = 640;
    public static final double TOP_TARGET_HEIGHT = 97;
    public static final double VERTICAL_FOV = 51.4;
    public static final double HORIZONTAL_FOV = 67;
    public static final double CAMERA_ANGLE = 13;
    public static final double TOP_CAMERA_HEIGHT = 14;

    // Axes
    public static final int AXIS_MOVE = 1;
    public static final int AXIS_ROTATE = 2;
    public static final int AXIS_THROTTLE = 3;

    // PID Constants
    public static final double KP_THIS_SHIT = 0;
    public static final double KD_THIS_SHIT = 0;
    public static final double TOLERANCE_THIS_SHIT = 0.1;

    public static final double KP_LIFT = -0.001;
    public static final double KI_LIFT = 0;
    public static final double KD_LIFT = 0;

    // Shooter Constants
    public static final double SPEED_SHOOTER = 1;
    public static final double SPEED_SHOOTER_KICKER = 1;

    public static final double TOLERANCE_LIFT = 5;
    public static final double ANGLE_LIFT_INCREMENT = 50;

}
