package com.github.circuitrunners;

/**
 * Useful Calib boys
 */
public class Calib {
/** Turns a -1 to 1 into 0 to 1 value */
    public static double throttleMath(double input){
        double output = (input+1)/2;
        return output;
    }
/**
 *  Adds deadband, then scales input by a factor linearly.
 *  Produces a straight line
 *  @param input: value to scale, -1 <= x <= 1
 *  @param min: deadband value, anything less will return 0, 0 < x <= 1
 *  @param scale: factor to scale down by, also max value, min < x < 1
 *  @return will max out at input = 1
 */
    public static double scaleLinear(double input, double min, double scale) {
        double output;
        if(Math.abs(input) < min) {
            output = 0;
        }
        else {
            output = scale * input;
        }
        return output;
    }

    /**
     * Adds deadband, then scales by a power, then scales by a constant, then sets a max value
     * Produces a radical-shaped curve
     * @param input: value to scale, -1 <= x <= 1
     * @param min: deadband value, anything less will return 0, 0 < x <= 1
     * @param scale factor to scale by, also the max value, min < x < 1
     * @param power power to scale by, 1 <= x
     * @return will max out at input = scale
     */
    public static double scaleDoubleFlat(double input, double min, double scale, double power){
        double output;
        if(Math.abs(input) < min) {
            output = 0;
        }
        else if(Math.abs(input) > scale){
            output = Math.signum(input) * scale;
        }
        else{
            output = scale * Math.signum(input) * Math.abs(Math.pow(input/scale,power));
        }
        return output;
    }

    /**
     * Adds deadband, then scales by a power, then scales by a constant
     * Produces an upwards concavity curve
     * @param input: value to scale, -1 <= x <= 1
     * @param min: deadband value, anything less will return 0, 0 < x <= 1
     * @param scale factor to scale by, also the max value, min < x < 1
     * @param power power to scale by, 1 <= x
     * @return will max out at input = 1
     */

    public static double scalePower(double input, double min, double scale, double power){
        double output;
        if(Math.abs(input) < min) {
            output = 0;
        }
        else {
            output = scale * Math.signum(input) * Math.abs(Math.pow(input,power));
        }
        return output;
    }
}
