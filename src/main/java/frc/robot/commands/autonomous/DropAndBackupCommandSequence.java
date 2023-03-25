// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autonomous;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.model.ClawPosition;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.DriveTrain;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class DropAndBackupCommandSequence extends SequentialCommandGroup {
  /** Creates a new DropAndBackupCommandSequence. */
  public DropAndBackupCommandSequence(Arm arm, Claw claw, DriveTrain driveTrain) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new MoveArmToPosition(arm, Constants.FAR_DROP_POSITION),
      new Forward(driveTrain, 1.0),
      new MoveClawToPosition(claw, ClawPosition.OPEN),
      new ParallelCommandGroup( // Move Arm to Home and Reverse Past Platform in parallel
        new MoveArmToPosition(arm, Constants.HOME_ARM_POSITION), 
        new Reverse(driveTrain, 5.25))
    );
  }
}
