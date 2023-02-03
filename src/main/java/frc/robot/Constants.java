// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }

    /* Robot Identifiers - CAN IDs */
    public static final int MOTOR_LEFT_1_ID = 3;
    public static final int MOTOR_LEFT_2_ID = 4;
    public static final int MOTOR_RIGHT_1_ID = 1;
    public static final int MOTOR_RIGHT_2_ID = 2;

    /* HCI Identifiers */
    public static final int RIGHT_TANK_JOYSTICK_USB_PORT = 0;
    public static final int LEFT_TANK_JOYSTICK_USB_PORT = 1;
    public static final int XBOX_CONTROLLER_USB_PORT = 2;

    private static final String TANK_DRIVE_LIMIT_KEY = "Tank Drive Limit";
    private static final double TANK_DRIVE_LIMITER_VALUE = 0.9;

    public static double getTankDriveLimiterValue() {
        return SmartDashboard.getNumber(TANK_DRIVE_LIMIT_KEY, TANK_DRIVE_LIMITER_VALUE);
    }

}
