package com.github.circuitrunners;

import com.github.circuitrunners.akilib.SmartDashboard2;
import com.github.circuitrunners.akilib.Xbox;
import com.github.circuitrunners.calib.CalibMath;
import com.github.circuitrunners.pipeline.button.ButtonPipeline;
import com.github.circuitrunners.system.Lift;
import com.github.circuitrunners.system.Shooter;
import com.github.circuitrunners.system.command.HomeLift;
import com.github.circuitrunners.system.command.Move;
import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.vision.AxisCamera;

public class Robot extends IterativeRobot {

    public static RobotDrive drive;
    public static Shooter shooter = new Shooter();
    public static Lift lift = new Lift();

    private Joystick joystick;
    private Xbox xbox;
    private ButtonPipeline pipeline;

    private AxisCamera camera;

    @Override
    public void robotInit() {
        SmartDashboard2.setNetwork(true);

        VictorSP frontLeft = new VictorSP(RobotValueMap.PORT_DRIVE_FRONT_LEFT);
        VictorSP rearLeft = new VictorSP(RobotValueMap.PORT_DRIVE_REAR_LEFT);
        VictorSP frontRight = new VictorSP(RobotValueMap.PORT_DRIVE_FRONT_RIGHT);
        VictorSP rearRight = new VictorSP(RobotValueMap.PORT_DRIVE_REAR_RIGHT);

        drive = new RobotDrive(frontLeft, rearLeft, frontRight, rearRight);
        drive.setExpiration(5);

        joystick = new Joystick(RobotValueMap.PORT_JOYSTICK);
        xbox = new Xbox(RobotValueMap.PORT_XBOX);
        pipeline = new ButtonPipeline(joystick, xbox);

        camera = new AxisCamera("10.10.2.11");
    }

    private final DigitalInput directionSwitch = new DigitalInput(1);
    private final DigitalInput timeoutSwitch = new DigitalInput(2);

    @Override
    public void autonomousInit() {
        Scheduler.getInstance().add(new HomeLift(timeoutSwitch.get() ? 3000 : 0));
        Lift.liftPid.disable();
        Scheduler.getInstance().add(new Move(directionSwitch.get() ? 0.8 : -0.8, -0.075, 0, 2400));
        Scheduler.getInstance().add(new Move(directionSwitch.get() ? 0.8 : -0.8, 0.075, 0, 2000));
    }

    @Override
    public void autonomousPeriodic() {}

    @Override
    public void teleopInit() {
        Lift.liftPid.enable();
    }

    @Override
    public void teleopPeriodic() {
        drive();
    }

    private void drive() {
        double moveVal = SmartDashboard2.put("moveVal", joystick.getY());
        double twistVal = SmartDashboard2.put("twistVal", joystick.getTwist());
        double throttleVal = SmartDashboard2.put("throttleVal", CalibMath.throttleMath(joystick.getThrottle()));

        double rotateVal = SmartDashboard2.put("rotateVal", CalibMath.scaleDoubleFlat(twistVal, SmartDashboard2.get("joystickDeadzone", RobotValueMap.JOYSTICK_DEADZONE), SmartDashboard2.get("joystickScaleFlat", RobotValueMap.JOYSTICK_SCALE_FLAT), SmartDashboard2.get("joystickScalePower", RobotValueMap.JOYSTICK_SCALE_POWER))); //could make magic numbers into constants but who cares
        double throttledMove = SmartDashboard2.put("throttledMove", throttleVal * moveVal);

        double throttledRotate = SmartDashboard2.put("throttledRotate", CalibMath.inverseAdjustedDeadband(throttleVal, 0.5) * rotateVal);
        drive.arcadeDrive(throttledMove, throttledRotate);
    }

    @Override
    public void disabledInit() {}
}
