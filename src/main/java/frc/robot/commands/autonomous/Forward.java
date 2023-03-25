// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autonomous;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrain;

public class Forward extends CommandBase {

  private final DriveTrain driveTrain;
  private final double duration;
  private final Timer timer;

  public Forward(DriveTrain driveTrain) {
    this(driveTrain, 5.0);
  }

  public Forward(DriveTrain driveTrain, double duration) {
    this(driveTrain, duration, new Timer());
  }

  /** Creates a new Reverse. */
  public Forward(DriveTrain driveTrain, double duration, Timer timer) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.driveTrain = driveTrain;
    this.duration = duration;
    this.timer = timer;
    addRequirements(driveTrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    driveTrain.moveLeftMotors(0.5);
    driveTrain.moveRightMotors(0.505);
    timer.reset();
    timer.start();
    System.out.println("** Moving Forward");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    timer.stop();
    System.out.println("** Moving Forward Complete");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return timer.hasElapsed(duration);
  }
}
