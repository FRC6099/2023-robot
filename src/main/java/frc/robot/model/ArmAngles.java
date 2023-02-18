// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.model;

/** Add your docs here. */
public class ArmAngles {
    private double lowerAngle;
    private double upperAngle;

    public ArmAngles(double lowerAngle, double upperAngle) {
        this.lowerAngle = lowerAngle;
        this.upperAngle = upperAngle;
    }

    public double getLowerAngle() {
        return this.lowerAngle;
    }

    public double getUpperAngle() {
        return this.upperAngle;
    }
}
