// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.model.ArmPosition;

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
  public static final int XBOX_CONTROLLER_USB_ID = 0;
  public static final int LEFT_JOYSTICK_USB_ID = 1;
  public static final int RIGHT_JOYSTICK_USB_ID = 2;


  /*** CAN IDs  ***/
  // Drive Train
  public static final int RIGHT_FRONT_DRIVE_MOTOR_CAN_ID=1;
  public static final int RIGHT_REAR_DRIVE_MOTOR_CAN_ID=2;
  public static final int LEFT_FRONT_DRIVE_MOTOR_CAN_ID=3;
  public static final int LEFT_REAR_DRIVE_MOTOR_CAN_ID=4;

  // Arm
  public static final int LOWER_ARM_MOTOR_CAN_ID = 5;
  public static final int UPPER_ARM_MOTOR_CAN_ID = 6;

  // Claw
  public static final int CLAW_MOTOR_CAN_ID = 7;


  /*** Arm Positions ***/
  // Angles
  public static final double START_LOWER_ARM_DEGREES = 101.5;
  public static final double START_UPPER_ARM_DEGREES = 21.5;

  // Boundaries
  public static final double MIN_HORIZONTAL_ARM_REACH = 12.0;       // INCHES
  public static final double MAX_HORIZONTAL_ARM_REACH = 69.0;       // INCHES (21.14" front of robot)
  public static final double MIN_VERTICAL_ARM_REACH = -12.0;        // INCHES (12.15" to ground)
  public static final double MAX_VERTICAL_ARM_REACH = 63.0;         // INCHES (12.15" deducted from 76")

  // Go to Positions
  public static final ArmPosition HOME_ARM_POSITION = new ArmPosition(30.51, 0.98);
  public static final ArmPosition FLOOR_PICKUP_POSITION = new ArmPosition(29.05, 9.33);
  public static final ArmPosition SHELF_PICKUP_POSITION = new ArmPosition(32.51, 31.57);
  public static final ArmPosition NEAR_DROP_POSITION = new ArmPosition(49.72, 30.51);
  public static final ArmPosition FAR_DROP_POSITION = new ArmPosition(65.07, 44.52);
}
