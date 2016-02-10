package com.github.circuitrunners;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.buttons.JoystickButton;

/**
 * Created by Owner on 2/9/2016.
 */
public class Xbox extends Joystick {

    private final int port;
    private final Joystick controller;
    public final JoystickButton a;
    public final JoystickButton b;
    public final JoystickButton x;
    public final JoystickButton y;
    public final JoystickButton start;
    public final JoystickButton back;
    public final JoystickButton leftBumper;
    public final JoystickButton rightBumper;


    public Xbox(final int port){
        super(port);
        this.port = port;
        this.controller = new Joystick(this.port);
        this.a = new JoystickButton(this.controller,1);
        this.b = new JoystickButton(this.controller,2);
        this.x = new JoystickButton(this.controller,3);
        this.y = new JoystickButton(this.controller,4);
        this.start = new JoystickButton(this.controller,7);
        this.back = new JoystickButton(this.controller,8);
        this.leftBumper = new JoystickButton(this.controller,5);
        this.rightBumper = new JoystickButton(this.controller,6);

    }

}
