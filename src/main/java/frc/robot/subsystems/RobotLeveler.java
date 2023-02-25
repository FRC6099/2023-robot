// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.ADIS16470_IMU;
import edu.wpi.first.wpilibj.ADIS16470_IMU.IMUAxis;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class RobotLeveler extends SubsystemBase {

  private final ADIS16470_IMU imu;
  /** Creates a new RobotLeveler. */
  public RobotLeveler() {
    this.imu = new ADIS16470_IMU();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public boolean makeRobotLevel() {
    this.imu.setYawAxis(IMUAxis.kX);
    double yaw = this.imu.getAngle();
    this.imu.setYawAxis(IMUAxis.kZ);
    double roll = this.imu.getAngle();
    this.imu.setYawAxis(IMUAxis.kY);
    double pitch = this.imu.getAngle();
    

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
    return false;
  }
}
