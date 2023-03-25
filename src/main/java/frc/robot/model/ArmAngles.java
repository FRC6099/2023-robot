// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.model;

/** Add your docs here. */
public class ArmAngles {
    private static final double LOWER_ARM_MIN_ANGLE_LIMIT = 40.0;
    private static final double LOWER_ARM_MAX_ANGLE_LIMIT = 101.5;
    private static final double UPPER_ARM_MIN_ANGLE_LIMIT = 20.0;
    private static final double UPPER_ARM_MAX_ANGLE_LIMIT = 170.0;

    private double lowerAngle;
    private double upperAngle;

    public ArmAngles(double lowerAngle, double upperAngle) {
        this.lowerAngle = lowerAngle;
        this.upperAngle = upperAngle;
    }

    public double getLowerAngle() {
        return getLimitedLowerAngle();
    }

    public double getUpperAngle() {
        return getLimitedUpperAngle();
    }

    private double getLimitedLowerAngle() {
        return Math.min(Math.max(this.lowerAngle, LOWER_ARM_MIN_ANGLE_LIMIT), LOWER_ARM_MAX_ANGLE_LIMIT);
    }

    private double getLimitedUpperAngle() {
        return Math.min(Math.max(this.upperAngle, UPPER_ARM_MIN_ANGLE_LIMIT), UPPER_ARM_MAX_ANGLE_LIMIT);
    }
}
