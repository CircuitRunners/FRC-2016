package com.github.circuitrunners.system;

import com.github.circuitrunners.RobotValueMap;
import com.github.circuitrunners.pipeline.button.handler.LiftButtonHandler;
import edu.wpi.first.wpilibj.CANTalon;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.command.Subsystem;

public class Lift extends Subsystem {
    public static CANTalon lift = new CANTalon(RobotValueMap.PORT_LIFT);
    public static PIDController liftPid;
    public static DigitalInput liftLimit = new DigitalInput(RobotValueMap.PORT_SENSOR_LIFT);

    public Lift() {
        liftPid = new PIDController(RobotValueMap.KP_LIFT, RobotValueMap.KI_LIFT, RobotValueMap.KD_LIFT, lift, lift);
        liftPid.setAbsoluteTolerance(RobotValueMap.TOLERANCE_LIFT);
        liftPid.setContinuous();
    }

    @Override
    protected void initDefaultCommand() {
        setDefaultCommand(new LiftButtonHandler());
    }
}
