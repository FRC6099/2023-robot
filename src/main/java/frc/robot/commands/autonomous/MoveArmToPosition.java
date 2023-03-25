// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autonomous;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.model.ArmPosition;
import frc.robot.subsystems.Arm;

public class MoveArmToPosition extends CommandBase {
  private final Arm arm;
  private final ArmPosition targetPosition;

  private boolean hasReachedPosition = false;
  /** Creates a new MoveArmToPosition. */
  public MoveArmToPosition(Arm arm, ArmPosition targetPosition) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(arm);
    this.arm = arm;
    this.targetPosition = targetPosition;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    hasReachedPosition = false;
    System.out.println("** Moving Arm to Position");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // Done when arm.goToPosition() returns true
    hasReachedPosition = arm.goToPosition(targetPosition);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    arm.stop();
    System.out.println("** Moving Arm to Position Complete");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return hasReachedPosition;
  }
}
