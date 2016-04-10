package com.github.circuitrunners.pipeline.button.handler;

import com.github.circuitrunners.Robot;
import com.github.circuitrunners.system.Lift;
import edu.wpi.first.wpilibj.command.Command;

public class LiftButtonHandler extends Command {

    private double setpoint;

    public LiftButtonHandler() {
        setpoint = 0;
    }

    public LiftButtonHandler(double setpoint) {
        this.setpoint = setpoint;
    }

    @Override
    protected void initialize() {
        requires(Robot.lift);
    }

    @Override
    protected void execute() {
        Lift.liftPid.setSetpoint(Lift.liftPid.get() + setpoint);
    }

    @Override
    protected boolean isFinished() {
        return !Lift.liftLimit.get();
    }

    @Override
    protected void end() {
        Lift.liftPid.setSetpoint(Lift.liftPid.get());
    }

    @Override
    protected void interrupted() {
        end();
    }
}
