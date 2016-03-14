package com.github.circuitrunners.commands;

import com.github.circuitrunners.subsystems.Shooter;
import edu.wpi.first.wpilibj.command.Command;

/**
 * Created by Akilan on 12.03.2016.
 */
public class Kick extends Command {

    private Shooter shooter = new Shooter();
    private Shooter.ShootDirection shootDirection = Shooter.ShootDirection.STOP;

    public Kick(Shooter.ShootDirection shootDirection) {
        this.shootDirection = shootDirection;
    }

    @Override
    protected void initialize() {
        requires(shooter);
    }

    @Override
    protected void execute() {
        shooter.kick(shootDirection);
    }

    @Override
    protected boolean isFinished() {
        return false;
    }

    @Override
    protected void end() {
        shooter.kick(Shooter.ShootDirection.STOP);
    }

    @Override
    protected void interrupted() {
    }
}