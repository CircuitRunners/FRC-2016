package com.github.circuitrunners.system.command;

import com.github.circuitrunners.Robot;
import edu.wpi.first.wpilibj.command.Command;

/**
 * Created by mastercoms on 4/10/16.
 */
public class Move extends Command {
    double moveVal;
    double rotateVal;
    int delay;
    int waitTime;

    public Move(double moveVal, double rotateVal) {
        this(moveVal, rotateVal, 0);
    }

    public Move(double moveVal, double rotateVal, int waitTime) {
        this.moveVal = moveVal;
        this.rotateVal = rotateVal;
        this.waitTime = waitTime;
    }

    public Move(double moveVal, double rotateVal, int delay, int waitTime) {
        this.moveVal = moveVal;
        this.rotateVal = rotateVal;
        this.delay = delay;
        this.waitTime = waitTime;
    }

    @Override
    protected void initialize() {
        if (delay > 0) {
            try {
                Thread.sleep(delay);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
        }
        Robot.drive.arcadeDrive(moveVal, rotateVal);
        if (waitTime > 0) {
            try {
                Thread.sleep(waitTime);
                Robot.drive.arcadeDrive(0, 0);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
        }
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
        Robot.drive.arcadeDrive(0, 0);
    }

    @Override
    protected void interrupted() {
        end();
    }
}
