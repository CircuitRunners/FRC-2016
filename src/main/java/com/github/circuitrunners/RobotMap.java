package com.github.circuitrunners;

import com.analog.adis16448.frc.ADIS16448_IMU;
import com.github.circuitrunners.akilib.Xbox;
import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.vision.AxisCamera;

public class RobotMap {

	// Motor ports
	public static final Talon[] motors = new Talon[]{ new Talon(0), new Talon(1),
                                                      new Talon(2), new Talon(3) };

    // Robot Drive
    public static final RobotDrive robotDrive = new RobotDrive(motors[0], motors[1],
                                                               motors[2],motors[3]);

	// Joystick ports
	public static final Joystick joystick = new Joystick(0);
    public static final Xbox xbox = new Xbox(1);

	// IMU
	public static final ADIS16448_IMU imu = new ADIS16448_IMU();
    static {
        imu.calibrate();
    }

	// Lift limit
	public static final DigitalInput liftLimit = new DigitalInput(0);

	// Lift motor
	public static final CANTalon liftMotor = new CANTalon(1);
    static {
        liftMotor.reverseSensor(true);
        liftMotor.reverseOutput(true);
    }

	// Kicker motor
	public static final CANTalon kickerMotor = new CANTalon(2);

	// Shooter wheels
	public static final VictorSP[] shooterWheelMotors = new VictorSP[]{ new VictorSP(4),
                                                                        new VictorSP(5) };

    // Camera
    public static final AxisCamera axisCamera = new AxisCamera("10.10.2.11");
}
