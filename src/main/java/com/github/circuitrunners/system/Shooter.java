package com.github.circuitrunners.system;

import com.github.circuitrunners.RobotValueMap;
import com.github.circuitrunners.pipeline.button.handler.ShooterButtonHandler;
import edu.wpi.first.wpilibj.CANTalon;
import edu.wpi.first.wpilibj.VictorSP;
import edu.wpi.first.wpilibj.command.Subsystem;

public class Shooter extends Subsystem {
    public static VictorSP shooterWheelLeft = new VictorSP(RobotValueMap.PORT_SHOOTER_LEFT);
    public static VictorSP shooterWheelRight = new VictorSP(RobotValueMap.PORT_SHOOTER_RIGHT);
    public static CANTalon shooterKicker = new CANTalon(RobotValueMap.PORT_SHOOTER_KICKER);

    @Override
    protected void initDefaultCommand() {
        setDefaultCommand(new ShooterButtonHandler());
    }
}
