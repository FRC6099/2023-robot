// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.model;

/** Add your docs here. */
public class ArmPosition {

    private static final double MAX_HORIZONTAL_REACH = 48.0;       // INCHES
    private static final double MAX_VERTICAL_REACH = 76.0;         // INCHES
    private static final double MIN_HORIZONTAL_REACH = 12.0;       // INCHES
    private static final double MIN_VERTICAL_REACH = -12.0;        // INCHES

    private double x;
    private double y;
    private ArmAngles armAngles;

    public ArmPosition(double x, double y, ArmAngles armAngles) {
        this.x = x;
        this.y = y;
        this.armAngles = armAngles;
    }

    public double getX() {
        return x;
    }

    public double getY() {
        return y;
    }

    public void setX(double x) {
        this.x = x;
    }

    public void setY(double y) {
        this.y = y;
    }

    public void addX(double x) {
        this.x = getLimitedValue(this.x, x, MAX_HORIZONTAL_REACH, MIN_HORIZONTAL_REACH);
    }

    public void addY(double y) {
        this.y = getLimitedValue(this.y, y, MAX_VERTICAL_REACH, MIN_VERTICAL_REACH);
    }

    public ArmAngles getArmAngles() {
        return armAngles;
    }

    private double getLimitedValue(double val, double addedVal, double max, double min) {
        if (val + addedVal > max) {
            return max;
        }
        if (val + addedVal < min) {
            return min;
        }
        return val + addedVal;
    }
}
