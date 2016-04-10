package com.github.circuitrunners.system.command;

import com.github.circuitrunners.system.Lift;
import edu.wpi.first.wpilibj.command.Command;

public class HomeLift extends Command {
    int timeout;

    public HomeLift(int homeTimeout){
        timeout = homeTimeout;

    }

    @Override
    protected void initialize() {
        long last = System.currentTimeMillis();
        long curr;
        long diff = 0;
        while (diff < timeout && Lift.liftLimit.get()) {
            Lift.lift.set(0.75);
            curr = System.currentTimeMillis();
            diff = curr - last;
        }
        Lift.lift.set(0);
        Lift.lift.setEncPosition(0);
    }

    @Override
    protected void execute() {

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

    }
}
