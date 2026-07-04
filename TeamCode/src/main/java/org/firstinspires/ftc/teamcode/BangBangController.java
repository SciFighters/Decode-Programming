package org.firstinspires.ftc.teamcode;

public class BangBangController {
    private double lowValue = 0.0;
    private double highValue = 1.0;
    private double tolerance = 0.0;

    public BangBangController(double tolerance) {
        this.tolerance = tolerance;
    }

    public void setTolerance(double tolerance) {
        this.tolerance = tolerance;
    }

    public void setHighValue(double highValue) {
        this.highValue = highValue;
    }

    public void setLowValue(double lowValue) {
        this.lowValue = lowValue;
    }

    public double calculate(double measurement, double setpoint) {
        if (Math.abs(measurement - setpoint) < tolerance) return lowValue;
        else if (measurement < setpoint) {
            return highValue;
        } else return lowValue;
    }

    public boolean isAtSetpoint(double measurement, double setpoint) {
        return Math.abs(measurement - setpoint) < tolerance;
    }
}
