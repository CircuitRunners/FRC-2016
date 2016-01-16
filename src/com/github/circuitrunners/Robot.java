
package com.github.circuitrunners;

import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Robot extends IterativeRobot {

    Joystick joystick;

    AnalogGyro gyro;

    VictorSP motors[];

    /* no pid for now
    ArrayList<PIDController> pids;
    */

    @Override
    public void robotInit() {
        joystick = new Joystick(0);
        gyro = new AnalogGyro(0);
        motors = new VictorSP[]{new VictorSP(0), new VictorSP(3), new VictorSP(4), new VictorSP(5)};
        /* no pid for now
        pids = new ArrayList<>();
        for (CANTalon motor : motors) {
            pids.add(new PIDController(1, 1, 1, 1, gyro, motor));
        }
        */
        motors[0].setInverted(true);
        motors[3].setInverted(true);
    }

    @Override
    public void autonomousInit() {

    }

    @Override
    public void autonomousPeriodic() {

    }

    @Override
    public void teleopInit() {

    }

    @Override
    public void teleopPeriodic() {
        double rotateVal = joystick.getTwist();
        if (joystick.getRawButton(1)) {
            gyro.reset();
        }

        if (0 <= rotateVal && rotateVal < 90) {
            motors[0].set(Math.sin(rotateVal));
            motors[1].set(Math.sin(rotateVal));
            motors[2].set(joystick.getMagnitude());
            motors[3].set(joystick.getMagnitude());
        } else if (90 <= rotateVal && rotateVal < 180) {
            motors[2].set(Math.sin(rotateVal));
            motors[3].set(Math.sin(rotateVal));
            motors[0].set(joystick.getMagnitude());
            motors[1].set(joystick.getMagnitude());
        } else if (180 <= rotateVal && rotateVal < 270){
            motors[2].set(Math.sin(rotateVal));
            motors[3].set(Math.sin(rotateVal));
            motors[0].set(-joystick.getMagnitude());
            motors[1].set(-joystick.getMagnitude());
        } else if (270 <= rotateVal && rotateVal < 360){
            motors[0].set(Math.sin(rotateVal));
            motors[1].set(Math.sin(rotateVal));
            motors[2].set(-joystick.getMagnitude());
            motors[3].set(-joystick.getMagnitude());
        } else {
            for (VictorSP motor : motors) {
                motor.set(0);
            }
        }
        SmartDashboard.putNumber("rotateVal", rotateVal);
        SmartDashboard.putData("test", gyro);
    }

    @Override
    public void testPeriodic() {

    }

}
