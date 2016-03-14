package com.github.circuitrunners.commands;

import com.github.circuitrunners.OI;
import com.github.circuitrunners.RobotMap;
import com.github.circuitrunners.subsystems.Drivebase;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.command.PIDCommand;

/**
 * Created by Akilan on 12.03.2016.
 */
public class ControlDrive extends PIDCommand {

    //PID Constants
    public static final double KP = 0;
    public static final double KI = 0;
    public static final double KD = 0;
    public static final double TOLERANCE_THIS_SHIT = 2;

    private static final double JOYSTICK_DEADZONE_PID = 0.2;

    private final Drivebase drivebase = new Drivebase();

    public PIDController pidController;
    private boolean triggerPressed;
    private double moveVal;
    private double twistVal;

    public ControlDrive() {
        super(KP, KI, KD);
        getPIDController().setAbsoluteTolerance(TOLERANCE_THIS_SHIT);
    }

    @Override
    protected void initialize() {
        requires(drivebase);
    }

    @Override
    protected void execute() {
        moveVal = -RobotMap.joystick.getY();
        twistVal = RobotMap.joystick.getTwist();

        if (OI.getInstance().buttonPidEnable.get()) {
            if (!getPIDController().isEnabled()) getPIDController().enable();
            if (!triggerPressed || twistVal > JOYSTICK_DEADZONE_PID) {
                setSetpoint(getPosition());
                triggerPressed = true;
            }
        } else {
            getPIDController().disable();
            triggerPressed = false;
        }

        if (!getPIDController().isEnabled()) drivebase.arcadeDrive();
    }

    @Override
    protected boolean isFinished() {
        return false;
    }

    @Override
    protected void end() {
    }

    @Override
    protected void interrupted() {
        drivebase.arcadeDrive(0,0);
    }

    @Override
    protected double returnPIDInput() {
        return RobotMap.imu.getAngle();
    }

    @Override
    protected void usePIDOutput(double output) {
        drivebase.arcadeDrive(moveVal, output);
    }
}