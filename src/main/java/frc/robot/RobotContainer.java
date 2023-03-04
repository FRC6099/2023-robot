// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.Autos;
import frc.robot.commands.ExampleCommand;
import frc.robot.commands.LevelRobot;
import frc.robot.commands.OperateArm;
import frc.robot.controllers.OperateArmController;
import frc.robot.model.ClawPosition;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.Handbrake;
import frc.robot.subsystems.Leveler;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();
  private final DriveTrain driveTrain = new DriveTrain();
  private final Arm arm = new Arm();
  private final Claw claw = new Claw();
  private final Leveler leveler = new Leveler();
  private final Handbrake handbrake = new Handbrake();

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController xboxController = new CommandXboxController(OperatorConstants.kDriverControllerPort);
  private final CommandJoystick leftJoystick = new CommandJoystick(Constants.LEFT_JOYSTICK_USB_ID);
  private final CommandJoystick rightJoystick = new CommandJoystick(Constants.RIGHT_JOYSTICK_USB_ID);

  private final OperateArmController operateArmController = new OperateArmController(xboxController);
  private final OperateArm operateArm = new OperateArm(arm, operateArmController);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();
    configureSubsystems();
  }

  /**
   * Used to configure Subsystem::setDefaultCommand. This will schedule a Command until 
   * another Command takes over from configureBindings() method which requires 
   * the Subsystem.
   * Default commands are best used for non-button input or setting default behavior.
   */
  private void configureSubsystems() {
    // this.mySubsystem.setDefaultCommand(myCommand);
    this.arm.setDefaultCommand(operateArm);
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`
    new Trigger(m_exampleSubsystem::exampleCondition)
        .onTrue(new ExampleCommand(m_exampleSubsystem));

    // Schedule `exampleMethodCommand` when the Xbox controller's B button is pressed,
    // cancelling on release.

    // ARM COMMANDS
    xboxController.a().whileTrue(new RunCommand(() -> arm.goToPosition(Constants.FLOOR_PICKUP_POSITION), arm));
    xboxController.y().whileTrue(new RunCommand(() -> arm.goToPosition(Constants.SHELF_PICKUP_POSITION), arm));
    xboxController.b().whileTrue(new RunCommand(() -> arm.goToPosition(Constants.NEAR_DROP_POSITION), arm));
    xboxController.x().whileTrue(new RunCommand(() -> arm.goToPosition(Constants.FAR_DROP_POSITION), arm));

    // CLAW COMMANDS
    xboxController.leftTrigger().whileTrue(new RunCommand(() -> claw.goToPosition(ClawPosition.OPEN), claw));
    xboxController.leftBumper().whileTrue(new RunCommand(() -> claw.goToPosition(ClawPosition.CUBE), claw));
    xboxController.rightBumper().whileTrue(new RunCommand(() -> claw.goToPosition(ClawPosition.CONE), claw));
    xboxController.rightTrigger().whileTrue(new RunCommand(() -> claw.goToPosition(ClawPosition.CLOSED), claw));

    // LEVELER COMMANDS
    leftJoystick.button(Constants.LEVELER_BUTTON_ID).whileTrue(new LevelRobot(leveler, driveTrain));

    // HANDBRAKE COMMANDS
    rightJoystick.button(Constants.HANDBRAKE_ENGAGE_BUTTON_ID).whileTrue(new RunCommand(() -> { 
      driveTrain.stop(); 
      handbrake.engage();
    }, handbrake, driveTrain));
    leftJoystick.button(Constants.HANDBRAKE_RELEASE_BUTTON_ID).whileTrue(new RunCommand(() -> 
      handbrake.release(), handbrake));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return Autos.exampleAuto(m_exampleSubsystem);
  }

  public void simulationInit() {
    arm.simulationInit();
    claw.simulationInit();
  }

  public void simulationPeriodic() {
    arm.simulationPeriodic();
    claw.simulationPeriodic();
  }
}
