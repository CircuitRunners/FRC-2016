package com.github.circuitrunners.commands;

import com.github.circuitrunners.RobotMap;
import com.github.circuitrunners.akilib.SmartDashboard2;
import com.github.circuitrunners.subsystems.Shooter;
import edu.wpi.first.wpilibj.command.PIDCommand;

/**
 * Created by Akilan on 12.03.2016.
 */
public class Lift extends PIDCommand {

    private static final double KP = 0.001;
    private static final double KI = 0;
    private static final double KD = 0;

    private static final double TOLERANCE = 20;

    private static final double ANGLE_LIFT_INCREMENT = 100;
    private static final double ANGLE_LIFT_REST = 0;

    private Shooter shooter = new Shooter();
    private Shooter.LiftDirection liftDirection = Shooter.LiftDirection.STOP;
    private double output = 0;

    public Lift(Shooter.LiftDirection liftDirection) {
        super(KP, KI, KD);
        getPIDController().setAbsoluteTolerance(TOLERANCE);
        setSetpoint(ANGLE_LIFT_REST);
        this.liftDirection = liftDirection;
    }

    @Override
    protected void initialize() {
        requires(shooter);
    }

    @Override
    protected void execute() {
        // Put sensors on SmartDashboard
        SmartDashboard2.put("Hall", RobotMap.liftLimit);

        switch (liftDirection) {
            case UP: setSetpointRelative(ANGLE_LIFT_INCREMENT);
            case DOWN: setSetpointRelative(-ANGLE_LIFT_INCREMENT);
            case RESET: setSetpoint(0);
        }
        SmartDashboard2.put("shooterAngle", getSetpoint());
    }

    @Override
    protected boolean isFinished() {
        return liftDirection == Shooter.LiftDirection.DOWN && !RobotMap.liftLimit.get() ||
                getPosition() <= 0;
    }

    @Override
    protected void end() {
        shooter.lift(0);
    }

    @Override
    protected void interrupted() {
        shooter.lift(0);
    }

    @Override
    protected double returnPIDInput() {
        return RobotMap.liftMotor.getEncPosition();
    }

    @Override
    protected void usePIDOutput(double output) {
        shooter.lift(output);
    }
}