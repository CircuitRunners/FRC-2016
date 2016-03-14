package com.github.circuitrunners.commands;

import com.github.circuitrunners.subsystems.Shooter;
import edu.wpi.first.wpilibj.command.Command;

/**
 * Created by Akilan on 12.03.2016.
 */
public class Shoot extends Command {

    private Shooter shooter = new Shooter();
    private Shooter.ShootDirection direction = Shooter.ShootDirection.STOP;

    public Shoot(Shooter.ShootDirection direction) {
        this.direction = direction;
    }

    @Override
    protected void initialize() {
        requires(shooter);
    }

    @Override
    protected void execute() {
        shooter.shootAndIntake(direction);
    }

    @Override
    protected boolean isFinished() {
        return false;
    }

    @Override
    protected void end() {
        shooter.shootAndIntake(Shooter.ShootDirection.STOP);
    }

    @Override
    protected void interrupted() {
        shooter.shootAndIntake(Shooter.ShootDirection.STOP);
    }
}