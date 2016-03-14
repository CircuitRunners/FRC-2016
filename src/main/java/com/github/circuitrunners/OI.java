package com.github.circuitrunners;

import com.github.circuitrunners.akilib.*;
import com.github.circuitrunners.commands.Kick;
import com.github.circuitrunners.commands.Lift;
import com.github.circuitrunners.commands.Shoot;
import com.github.circuitrunners.subsystems.Shooter;
import edu.wpi.first.wpilibj.buttons.Button;
import edu.wpi.first.wpilibj.buttons.JoystickButton;

public class OI {

    // Static instance
    private static final OI INSTANCE = new OI();
    public static OI getInstance() {
        return INSTANCE;
    }

    // Buttons
    public Button buttonPidEnable;
    public Button buttonGyroReset;

    public Button buttonShooterLiftDown;
    public Button buttonShooterLiftUp;
    public Button resetLift;

    public Button buttonShooterWheelspinOut;
    public Button buttonShooterWheelspinIn;
    public Button buttonShooterKickOut;
    public Button buttonShooterKickIn;
    
    public OI() {
        buttonPidEnable = new JoystickButton(RobotMap.joystick, 4);
        buttonGyroReset = new JoystickButton(RobotMap.joystick, 6);
        switch (SmartDashboard2.get("driverControlType", "Joystick+Xbox")) {
            case "Joystick+Xbox":
                buttonShooterLiftDown = new XboxButton(RobotMap.xbox, Xbox.Button.A);
                buttonShooterLiftUp = new XboxButton(RobotMap.xbox, Xbox.Button.B);
                resetLift = new XboxButton(RobotMap.xbox, Xbox.Button.BACK);

                buttonShooterWheelspinOut = new XboxButton(RobotMap.xbox, Xbox.Button.RIGHT_BUMPER);
                buttonShooterWheelspinIn = new JoystickButton(RobotMap.joystick, 1);
                buttonShooterKickIn = new JoystickButton(RobotMap.xbox, Xbox.Button.X.ordinal());
                buttonShooterKickOut = new JoystickButton(RobotMap.xbox, Xbox.Button.Y.ordinal());
                break;
            default:
                buttonShooterLiftDown = new JoystickButton(RobotMap.joystick, 3);
                buttonShooterLiftUp = new JoystickButton(RobotMap.joystick, 5);
                resetLift = new JoystickButton(RobotMap.joystick, 9);

                buttonShooterWheelspinOut = new JoystickButton(RobotMap.joystick, 1);
                buttonShooterWheelspinIn = new JoystickButton(RobotMap.joystick, 2);
                buttonShooterKickOut = new Button2(RobotMap.joystick, POVDirection.UP);
                buttonShooterKickIn = new Button2(RobotMap.joystick, POVDirection.DOWN);
        }

        buttonShooterLiftDown.whileHeld(new Lift(Shooter.LiftDirection.DOWN));
        buttonShooterLiftUp.whileHeld(new Lift(Shooter.LiftDirection.UP));
        resetLift.whileHeld(new Lift(Shooter.LiftDirection.RESET));

        buttonShooterWheelspinIn.whileHeld(new Shoot(Shooter.ShootDirection.IN));
        buttonShooterWheelspinOut.whileHeld(new Shoot(Shooter.ShootDirection.OUT));
        buttonShooterKickIn.whileHeld(new Kick(Shooter.ShootDirection.IN));
        buttonShooterKickOut.whileHeld(new Kick(Shooter.ShootDirection.OUT));
    }
}
