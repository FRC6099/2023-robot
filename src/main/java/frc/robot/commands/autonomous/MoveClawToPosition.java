// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autonomous;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.model.ClawPosition;
import frc.robot.subsystems.Claw;

public class MoveClawToPosition extends CommandBase {
  private final Claw claw;
  private final ClawPosition targetPosition;
  private final double duration;
  private final Timer timer;

  private boolean hasReachedPosition = false;
  /** Creates a new MoveClawToPosition. */
  public MoveClawToPosition(Claw claw, ClawPosition targetPosition) {
    this(claw, targetPosition, 2.0);
  }
  
  public MoveClawToPosition(Claw claw, ClawPosition targetPosition, double duration) {
    addRequirements(claw);
    this.claw = claw;
    this.targetPosition = targetPosition;
    this.duration = duration;
    this.timer = new Timer();
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    hasReachedPosition = false;
    timer.reset();
    timer.start();
    System.out.println("** Moving Claw to Position");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    hasReachedPosition = claw.goToPosition(targetPosition);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    timer.stop();
    claw.stop();
    System.out.println("** Moving Claw to Position Complete");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return timer.hasElapsed(duration);
  }
}
