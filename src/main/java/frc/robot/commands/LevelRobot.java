// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.model.LevelMovement;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Leveler;

public class LevelRobot extends CommandBase {
  private final Leveler leveler;
  private final DriveTrain driveTrain;

  /** Creates a new LevelRobot. */
  public LevelRobot(Leveler leveler, DriveTrain driveTrain) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(leveler);
    this.leveler = leveler;
    this.driveTrain = driveTrain;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    LevelMovement movement = leveler.getLevelMovement();
    switch (movement) {
      case LEFT:
        driveTrain.moveLeftMotors(0.2);
        driveTrain.moveRightMotors(-0.2);
        break;
      case RIGHT:
        driveTrain.moveLeftMotors(-0.2);
        driveTrain.moveRightMotors(0.2);
        break;
      case FORWARD:
        driveTrain.moveLeftMotors(0.2);
        driveTrain.moveRightMotors(0.2);
        break;
      case REVERSE:
        driveTrain.moveLeftMotors(-0.2);
        driveTrain.moveRightMotors(-0.2);
        break;
      case STOP:
      default:
      driveTrain.stop();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    driveTrain.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return leveler.isLevel();
  }
}
