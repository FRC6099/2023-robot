// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.model.ArmPosition;
import frc.robot.model.ClawPosition;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

  // Example from FRC - not a good practice to nest classes:
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }


  /*** USB Ports  ***/
  public static final boolean ENABLE_CAMERAS = true;
  public static final int XBOX_CONTROLLER_USB_ID = 0;
  public static final int LEFT_JOYSTICK_USB_ID = 1;
  public static final int RIGHT_JOYSTICK_USB_ID = 2;


  /*** Joystick Mappings ***/
  public static final int DRIVETRAIN_MICRO_ADJUSTER_BUTTON_ID = 1;
  public static final int LEVELER_BUTTON_ID = 11;
  public static final int HANDBRAKE_ENGAGE_BUTTON_ID = 12;
  public static final int HANDBRAKE_RELEASE_BUTTON_ID = 12;

  /*** CAN IDs  ***/
  // Drive Train
  public static final int RIGHT_FRONT_DRIVE_MOTOR_CAN_ID = 1;
  public static final int RIGHT_REAR_DRIVE_MOTOR_CAN_ID = 2;
  public static final int LEFT_FRONT_DRIVE_MOTOR_CAN_ID = 3;
  public static final int LEFT_REAR_DRIVE_MOTOR_CAN_ID = 4;

  // Arm
  public static final int LOWER_ARM_MOTOR_CAN_ID = 5;
  public static final int UPPER_ARM_MOTOR_CAN_ID = 6;

  // Claw
  public static final int CLAW_MOTOR_CAN_ID = 7;

  // Handbrake
  public static final int RIGHT_HANDBRAKE_CAN_ID = 8;
  public static final int LEFT_HANDBRAKE_CAN_ID = 9;


  /*** DIO IDs ***/
  public static final int LOWER_ARM_MAX_LIMIT_SWITCH_ID = 0;
  public static final int LOWER_ARM_MIN_LIMIT_SWITCH_ID = 1;
  public static final int UPPER_ARM_LIMIT_SWITCH_ID = 2;
  public static final int ARM_MAX_ANGLE_POTENTIOMETER_ID = 0;

  /*** Arm Positions ***/
  // Angles
  public static final double START_LOWER_ARM_DEGREES = 96.0;
  public static final double START_UPPER_ARM_DEGREES = 24.5;

  // Boundaries
  public static final double MIN_HORIZONTAL_ARM_REACH = 12.0;       // INCHES
  public static final double MAX_HORIZONTAL_ARM_REACH = 69.0;       // INCHES (21.14" front of robot)
  public static final double MIN_VERTICAL_ARM_REACH = -12.0;        // INCHES (12.15" to ground)
  public static final double MAX_VERTICAL_ARM_REACH = 63.0;         // INCHES (12.15" deducted from 76")

  // Go to Positions
  public static final ArmPosition HOME_ARM_POSITION = new ArmPosition(18.4395, -1.8956);         // TICKS: Lower Ticks: 8537.0, Upper Ticks: 2655.0
  public static final ArmPosition FLOOR_PICKUP_POSITION = new ArmPosition(29.05, 9.33);
  public static final ArmPosition SHELF_PICKUP_POSITION = new ArmPosition(32.51, 31.57);
  public static final ArmPosition NEAR_DROP_POSITION = new ArmPosition(44.3798, 35.0731);    // TICKS: Lower Ticks: 7438.0, Upper Ticks: 10001.0
  public static final ArmPosition FAR_DROP_POSITION = new ArmPosition(68.0, 21.5);     // TICKS: Lower Ticks: 4492.0, Upper Ticks: 13387.0

  public static final boolean ARM_LOGGING = true;

  /*** Claw Positions ***/
  private static SendableChooser<ClawPosition> clawPositions = new SendableChooser<>();
  public static ClawPosition getStartingClawPosition() {
    ClawPosition selected = clawPositions.getSelected();
    if (selected == null) {
      return ClawPosition.CONE;
    }
    return selected;
  }

  /*** Leveler ***/
  public static final double LEVELER_ROLL_VARIANCE = 2.0;
  public static final double LEVELER_PITCH_VARIANCE = 2.0;


  /*** Dashboard ***/
  private static final String STARTING_CLAW_POSITION_KEY = "Start Claw Pos";
  private static final String ARM_SINGLE_AXIS_CONTROLLED_KEY = "Arm Single Axis";
  private static final String CLAW_SPEED_CONTROLLED_KEY = "Direct Claw";

  public static void initDashboard() {
    clawPositions.setDefaultOption(ClawPosition.CONE.name(), ClawPosition.CONE);
    clawPositions.addOption(ClawPosition.CLOSED.name(), ClawPosition.CLOSED);
    clawPositions.addOption(ClawPosition.OPEN.name(), ClawPosition.OPEN);
    clawPositions.addOption(ClawPosition.CUBE.name(), ClawPosition.CUBE);
    SmartDashboard.putData(STARTING_CLAW_POSITION_KEY, clawPositions);
    SmartDashboard.putBoolean(ARM_SINGLE_AXIS_CONTROLLED_KEY, true);
    SmartDashboard.putBoolean(CLAW_SPEED_CONTROLLED_KEY, true);
  }

  public static boolean isArmSingleAxisControl() {
    return SmartDashboard.getBoolean(ARM_SINGLE_AXIS_CONTROLLED_KEY, true);
    // return true;
  }

  public static boolean isClawSpeedControlled() {
    return SmartDashboard.getBoolean(CLAW_SPEED_CONTROLLED_KEY, true);
    // return true;
  }
}
