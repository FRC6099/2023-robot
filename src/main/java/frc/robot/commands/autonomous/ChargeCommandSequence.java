// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autonomous;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ChargeCommandSequence extends SequentialCommandGroup {
  /** Creates a new ChargeCommandSequence. */
  public ChargeCommandSequence() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      // Move Arm to Drop Position
      // Open Claw
      // Move Arm to Home Position
      // Backup (should exit home region)
      // Move Forward until not level
      // Level
      // Apply Handbrake
      // Stop
    );
  }
}
