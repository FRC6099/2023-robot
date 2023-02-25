// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.model.LevelMovement;
import frc.robot.subsystems.thirdparty.ADIS16470_IMU;
import frc.robot.subsystems.thirdparty.ADIS16470_IMU.IMUAxis;

public class RobotLeveler extends SubsystemBase {

  private final ADIS16470_IMU imu;
  private final IMUAxis yawAxis;
  private final IMUAxis rollAxis;
  private final IMUAxis pitchAxis;
  /** Creates a new RobotLeveler. */
  public RobotLeveler() {
    this.imu = new ADIS16470_IMU();
    this.yawAxis = imu.getYawAxis();
    this.rollAxis = imu.getRollAxis();
    this.pitchAxis = imu.getPitchAxis();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public LevelMovement getLevelMovement() {
    // TODO: This needs to be heavily evaluated
    //  https://www.chiefdelphi.com/t/adis-16470-wpilibj-class-mystery-why-can-we-only-get-one-axis-at-a-time/425461
    //  https://www.chiefdelphi.com/t/analog-devices-adis-16470-eng-basic-java-gyro-code-help/422553
    double yaw = this.imu.getAngle(yawAxis) % 360.0;
    double roll = this.imu.getAngle(rollAxis) % 360.0;
    double pitch = this.imu.getAngle(pitchAxis) % 360.0;
    

    // To be level:
    //   ROLL - 0 degrees
    //   PITCH - 0 degrees
    //   yaw - 0 to 360 degrees

    // Retrieve x/y/z from subsystem
    //      Automate Command to level drive train, when level stop (if problem leveling, add timeout)
    //      OR
    //      Execute Command when pressing button to level drive train (if problem leveling, release button)


    // FACT: When (yaw > 0 AND yaw < 180) OR (yaw > 180 < 360), ROLL will be > 0 degrees when traversing incline
    // If ROLL < 360 AND ROLL > 315, turn clockwise
    // If ROLL > 0   AND ROLL < 45,  turn counter-clockwise
    // If PITCH > 0   AND PITCH < 45,  move forward
    // If PICTH < 360 AND PITCH > 315, move backward

    if (roll < 360.0 && roll > 315.0) {
      return LevelMovement.RIGHT;
    } else if (roll > 0.0 && roll < 45.0) {
      return LevelMovement.LEFT;
    } else if (pitch > 0.0 && pitch < 45.0) {
      return LevelMovement.FORWARD;
    } else if (pitch < 360.0 && pitch > 315.0) {
      return LevelMovement.REVERSE;
    }

    return LevelMovement.STOP;
  }

  public boolean isLevel() {
    return this.getLevelMovement() == LevelMovement.STOP;
  }
}
