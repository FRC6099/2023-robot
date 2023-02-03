// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.commands.ExampleCommand;
import frc.robot.commands.TankDrive;
import frc.robot.controllers.TankDriveController;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.ExampleSubsystem;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitCommand;
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

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController xboxController = new CommandXboxController(Constants.XBOX_CONTROLLER_USB_PORT);
  private final CommandJoystick leftJoystick = new CommandJoystick(Constants.LEFT_TANK_JOYSTICK_USB_PORT);
  private final CommandJoystick rightJoystick = new CommandJoystick(Constants.RIGHT_TANK_JOYSTICK_USB_PORT);
  private final TankDriveController tankDriveController = new TankDriveController(leftJoystick, rightJoystick);


  // The robot's subsystems and commands are defined here...
  private final ExampleSubsystem exampleSubsystem = new ExampleSubsystem();
  private final DriveTrain driveTrain = new DriveTrain();

  // Command Classes
  private final ExampleCommand exampleCommand = new ExampleCommand(exampleSubsystem);
  private final TankDrive tankDrive = new TankDrive(driveTrain, tankDriveController);

  // Add ability to choose autonomous mode in SmartDashboard
  private final SendableChooser<Command> autonomousChooser = new SendableChooser<>();


  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    this.configureBindings();
    this.configureSubsystems();
    this.configureAutonomousModes();
  }

    /**
   * Used to configure Subsystem::setDefaultCommand. This will schedule a Command until 
   * another Command takes over from configureBindings() method which requires 
   * the Subsystem.
   * Default commands are best used for non-button input or setting default behavior.
   */
  private void configureSubsystems() {
    this.driveTrain.setDefaultCommand(tankDrive);
  }

  private void configureAutonomousModes() {
    this.autonomousChooser.setDefaultOption("Shoot and Backup", new WaitCommand(10.0));
    SmartDashboard.putData(this.autonomousChooser);
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

    // Schedule `exampleMethodCommand` when the Xbox controller's B button is pressed,
    // cancelling on release.
    xboxController.b().whileTrue(exampleSubsystem.exampleMethodCommand());
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    //return Autos.exampleAuto(m_exampleSubsystem);
    return autonomousChooser.getSelected();
  }
}
