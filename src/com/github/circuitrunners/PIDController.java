package com.github.circuitrunners;

import edu.wpi.first.wpilibj.livewindow.LiveWindowSendable;
import edu.wpi.first.wpilibj.tables.ITable;
import edu.wpi.first.wpilibj.tables.ITableListener;

/**
 * Created by Akilan on 16.01.2016.
 */
public class PIDController implements LiveWindowSendable {

    private double kP;
    private double kI;
    private double kD;
    private double setpoint;
    private double error;
    private double totalError;
    private boolean isAccumClearEnabled;
    private double inputUpperLimit;
    private double inputLowerLimit;
    private double outputUpperLimit = 1;
    private double outputLowerLimit = -1;

    public void PIDController (double kP, double kI, double kD, double setpoint) {
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
        this.setpoint = setpoint;
    }

    public double calculate(double sourceValue) {
        error = setpoint - sourceValue;
        double value = error * kP;

        totalError += error;
        value += totalError * kI;

        return value;
    }

    public void clearAccumulation() {
        totalError = 0;
    }

    public double getkP() {
        return kP;
    }

    public void setkP(double kP) {
        this.kP = kP;
    }

    public double getkI() {
        return kI;
    }

    public void setkI(double kI) {
        this.kI = kI;
    }

    public double getkD() {
        return kD;
    }

    public void setkD(double kD) {
        this.kD = kD;
    }

    public double getSetpoint() {
        return setpoint;
    }

    public void setSetpoint(double setpoint) {
        this.setpoint = setpoint;
    }

    public double getError() {
        return error;
    }

    public double getInputUpperLimit() {
        return inputUpperLimit;
    }

    public void setInputUpperLimit(double inputUpperLimit) {
        this.inputUpperLimit = inputUpperLimit;
    }

    public double getInputLowerLimit() {
        return inputLowerLimit;
    }

    public void setInputLowerLimit(double inputLowerLimit) {
        this.inputLowerLimit = inputLowerLimit;
    }

    public double getOutputUpperLimit() {
        return outputUpperLimit;
    }

    public void setOutputUpperLimit(double outputUpperLimit) {
        this.outputUpperLimit = outputUpperLimit;
    }

    public double getOutputLowerLimit() {
        return outputLowerLimit;
    }

    public void setOutputLowerLimit(double outputLowerLimit) {
        this.outputLowerLimit = outputLowerLimit;
    }

    private ITable table;
    private ITableListener listener;

    @Override
    public void updateTable() {
        if (table != null) {
            table.putNumber("error", error);
            table.putNumber("totalError", totalError);
        }
    }

    @Override
    public void startLiveWindowMode() {
        listener = new ITableListener() {
            public void valueChanged(ITable itable, String key, Object value, boolean bln) {
                if (key == "kP" || key == "kI" || key == "kD" || key == "setpoint") {
                    try {
                        getClass().getField(key).setDouble(this, (Double) value);
                    } catch (NoSuchFieldException | IllegalAccessException e) {
                        e.printStackTrace();
                    }
                } else if (key == "isAccumClearEnabled") {
                    try {
                        getClass().getField(key).setBoolean(this, (Boolean) value);
                    } catch (IllegalAccessException | NoSuchFieldException e) {
                        e.printStackTrace();
                    }
                }
            }
        };
        table.addTableListener(listener, true);
    }

    @Override
    public void stopLiveWindowMode() {
        table.removeTableListener(listener);
    }

    @Override
    public void initTable(ITable subtable) {
        table = subtable;
        updateTable();
    }

    @Override
    public ITable getTable() {
        return table;
    }

    @Override
    public String getSmartDashboardType() {
        return "PID";
    }
}
