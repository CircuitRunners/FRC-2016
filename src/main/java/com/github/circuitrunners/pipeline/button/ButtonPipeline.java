package com.github.circuitrunners.pipeline.button;

import com.github.circuitrunners.RobotValueMap;
import com.github.circuitrunners.akilib.Button2;
import com.github.circuitrunners.akilib.POVDirection;
import com.github.circuitrunners.pipeline.button.handler.LiftButtonHandler;
import com.github.circuitrunners.pipeline.button.handler.LiftSaveButtonHandler;
import com.github.circuitrunners.pipeline.button.handler.ShooterButtonHandler;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.buttons.JoystickButton;

public class ButtonPipeline {
    public ButtonPipeline(GenericHID... joysticks) {
        /* --- Button maps --- */
        /* Drive */
        JoystickButton drive180 = new JoystickButton(joysticks[0], 3);
        /* Lift */
        JoystickButton liftZero = new JoystickButton(joysticks[1], 1);
        JoystickButton liftSaveReturnToggle = new JoystickButton(joysticks[1], 8);
        Button2 liftDown = new Button2(joysticks[1], POVDirection.DOWN);
        Button2 liftUp = new Button2(joysticks[1], POVDirection.UP);
        /* Shooter */
        JoystickButton shooterOut = new JoystickButton(joysticks[1], 5);
        JoystickButton shooterOutSlow = new JoystickButton(joysticks[1], 6);
        JoystickButton shooterIn = new JoystickButton(joysticks[0], 1);

        /* --- Bind buttons to events --- */
        shooterIn.whileHeld(new ShooterButtonHandler(RobotValueMap.SPEED_SHOOTER, true));
        shooterOutSlow.whileHeld(new ShooterButtonHandler(RobotValueMap.SPEED_SHOOTER * 0.5, false));
        shooterOut.whileHeld(new ShooterButtonHandler(RobotValueMap.SPEED_SHOOTER, false));
        liftUp.whileHeld(new LiftButtonHandler(RobotValueMap.ANGLE_LIFT_INCREMENT));
        liftDown.whileHeld(new LiftButtonHandler(-RobotValueMap.ANGLE_LIFT_INCREMENT));
        liftSaveReturnToggle.whenPressed(new LiftSaveButtonHandler());
        liftZero.whenPressed(new LiftButtonHandler());
        //drive180.whenPressed();
    }
}
