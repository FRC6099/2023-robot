// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.controllers.OperateArmController;
import frc.robot.subsystems.Arm;

public class OperateArm extends CommandBase {

  private final Arm arm;
  private final OperateArmController controller;

  /** Creates a new OperateArm. */
  public OperateArm(Arm arm, OperateArmController controller) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(arm);
    this.arm = arm;
    this.controller = controller;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double changeInX = controller.getLeftPosition();
    double changeInY = controller.getRightPosition();

    if (Math.abs(changeInX) < 0.10) { changeInX = 0; } /* deadband 10% */
		if (Math.abs(changeInY) < 0.10) { changeInY = 0; } /* deadband 10% */

    if (changeInX == 0 && changeInY == 0) {
      arm.stop();
    } else {
      moveArm(changeInX, changeInY);
    }
  }

  private void moveArm(double x, double y) {
    if (Constants.isArmSingleAxisControl()) {
      arm.moveLowerArm(x * 0.5);
      arm.moveUpperArm(y * 0.5);
    } else {
      arm.addPosition(x*10.0, y*10.0);
    }
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
