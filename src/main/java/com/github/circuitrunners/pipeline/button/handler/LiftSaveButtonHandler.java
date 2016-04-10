package com.github.circuitrunners.pipeline.button.handler;

import com.github.circuitrunners.Robot;
import com.github.circuitrunners.system.Lift;
import edu.wpi.first.wpilibj.command.Command;

public class LiftSaveButtonHandler extends Command {
    public static boolean saved;
    public static double setpoint;

    public LiftSaveButtonHandler() {
        saved = !saved;
    }


    @Override
    protected void initialize() {
        requires(Robot.lift);
        if (saved) {
            Lift.liftPid.setSetpoint(setpoint);
        } else {
            setpoint = Lift.liftPid.get();
        }
    }

    @Override
    protected void execute() {}

    @Override
    protected boolean isFinished() {
        return false;
    }

    @Override
    protected void end() {}

    @Override
    protected void interrupted() {}
}
