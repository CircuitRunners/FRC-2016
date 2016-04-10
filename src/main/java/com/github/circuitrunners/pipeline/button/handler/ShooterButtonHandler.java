package com.github.circuitrunners.pipeline.button.handler;

import com.github.circuitrunners.RobotValueMap;
import com.github.circuitrunners.system.Shooter;
import edu.wpi.first.wpilibj.command.Command;

import static com.github.circuitrunners.Robot.shooter;

public class ShooterButtonHandler extends Command {
    private double speed;
    private boolean reverse;

    public ShooterButtonHandler() {}

    public ShooterButtonHandler(double speed, boolean reverse) {
        this.speed = speed;
        this.reverse = reverse;
    }

    @Override
    protected void initialize() {
        requires(shooter);
    }

    @Override
    protected void execute() {
        if (reverse) {
            setWheels();
            timeIntake();
            setKicker();
        } else {
            setKicker();
            timeIntake();
            setWheels();
        }
    }

    private void setWheels() {
        int flipper = reverse ? 1 : -1;
        Shooter.shooterWheelLeft.set(flipper * speed);
        Shooter.shooterWheelRight.set(-flipper * speed);
    }

    private void setKicker() {
        if (speed == 0) {
            Shooter.shooterKicker.set(0);
        } else {
            int flipper = reverse ? 1 : -1;
            Shooter.shooterKicker.set(flipper * RobotValueMap.SPEED_SHOOTER_KICKER);
        }
    }

    private void timeIntake() {
        int waitTime = reverse ? 500 : 200;
        try {
            Thread.sleep(waitTime);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
    }

    @Override
    protected boolean isFinished() {
        return false;
    }

    @Override
    protected void end() {
        speed = 0;
        setWheels();
        setKicker();
    }

    @Override
    protected void interrupted() {
        end();
    }
}
