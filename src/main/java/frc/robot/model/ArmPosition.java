// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.model;

import frc.robot.Constants;

/** Add your docs here. */
public class ArmPosition {

    private double x;
    private double y;
    private ArmAngles armAngles;

    public ArmPosition(double x, double y) {
        this(x, y, null);
    }

    public ArmPosition(double x, double y, ArmAngles armAngles) {
        this.x = 0.0;
        this.y = 0.0;
        this.addX(x);
        this.addY(y);
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
        this.x = getLimitedValue(this.x, x, Constants.MIN_HORIZONTAL_ARM_REACH, Constants.MAX_HORIZONTAL_ARM_REACH);
    }

    public void addY(double y) {
        this.y = getLimitedValue(this.y, y, Constants.MIN_VERTICAL_ARM_REACH, Constants.MAX_VERTICAL_ARM_REACH);
    }

    public ArmAngles getArmAngles() {
        return armAngles;
    }

    private double getLimitedValue(double val, double addedVal, double min, double max) {
        if (val + addedVal > max) {
            return max;
        }
        if (val + addedVal < min) {
            return min;
        }
        return val + addedVal;
    }
}
