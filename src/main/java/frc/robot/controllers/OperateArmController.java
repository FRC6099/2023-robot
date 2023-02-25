// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.controllers;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

/** Add your docs here. */
public class OperateArmController {
    private final CommandXboxController xboxController;
    public OperateArmController(CommandXboxController xboxController) {
        this.xboxController = xboxController;
    }

    public double getLeftPosition() {
        return this.xboxController.getRawAxis(1) * -1.0;
    }

    public double getRightPosition() {
        return this.xboxController.getRawAxis(5) * -1.0;
    }
}
