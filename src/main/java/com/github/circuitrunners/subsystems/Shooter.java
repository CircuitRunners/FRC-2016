package com.github.circuitrunners.subsystems;

import com.github.circuitrunners.RobotMap;
import edu.wpi.first.wpilibj.command.Subsystem;

/**
 * Created by Akilan on 12.03.2016.
 */
public class Shooter extends Subsystem {

    private static final double SPEED_SHOOTER_WHEEL_LEFT = 1;
    private static final double SPEED_SHOOTER_WHEEL_RIGHT = 1;

    private static final double SPEED_SHOOTER_KICKER_OUT = 1;
    private static final double SPEED_SHOOTER_KICKER_IN = 1;
    private static final double SPEED_SHOOTER_KICKER_REST = 0.3;

    public void lift(double speed) {
        RobotMap.liftMotor.set(speed);
    }

    public void shootAndIntake(ShootDirection shootDirection) {
        switch (shootDirection) {
            case IN:
                RobotMap.kickerMotor.set(SPEED_SHOOTER_KICKER_IN);
                RobotMap.shooterWheelMotors[0].set(SPEED_SHOOTER_WHEEL_LEFT);
                RobotMap.shooterWheelMotors[1].set(-SPEED_SHOOTER_WHEEL_RIGHT);
            case OUT:
                RobotMap.shooterWheelMotors[0].set(-SPEED_SHOOTER_WHEEL_LEFT);
                RobotMap.shooterWheelMotors[1].set(SPEED_SHOOTER_WHEEL_RIGHT);
            default:
                RobotMap.shooterWheelMotors[0].set(0);
                RobotMap.shooterWheelMotors[1].set(0);

        }
    }

    public void kick(ShootDirection shootDirection) {
        switch (shootDirection) {
            case IN: RobotMap.kickerMotor.set(SPEED_SHOOTER_KICKER_IN);
            case OUT: RobotMap.kickerMotor.set(-SPEED_SHOOTER_KICKER_OUT);
            default: RobotMap.kickerMotor.set(-SPEED_SHOOTER_KICKER_REST);
        }
    }

    public enum LiftDirection {
        UP, DOWN, RESET, STOP
    }

    public enum ShootDirection {
        IN, STOP, OUT
    }

    @Override
    public void initDefaultCommand() {
    }
}