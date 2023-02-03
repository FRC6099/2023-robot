// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class DriveTrain extends SubsystemBase {
  private final WPI_VictorSPX motorLeft1;
  private final WPI_VictorSPX motorLeft2;
  private final WPI_VictorSPX motorRight1;
  private final WPI_VictorSPX motorRight2;

  /** Creates a new DriveTrain. */
  public DriveTrain() {
    this(new WPI_VictorSPX(Constants.MOTOR_LEFT_1_ID),
            new WPI_VictorSPX(Constants.MOTOR_LEFT_2_ID),
            new WPI_VictorSPX(Constants.MOTOR_RIGHT_1_ID),
            new WPI_VictorSPX(Constants.MOTOR_RIGHT_2_ID));
  }

  public DriveTrain(WPI_VictorSPX motorLeft1, WPI_VictorSPX motorLeft2, WPI_VictorSPX motorRight1, WPI_VictorSPX motorRight2) {
    this.motorLeft1 = motorLeft1;
    this.motorLeft2 = motorLeft2;
    this.motorRight1 = motorRight1;
    this.motorRight2 = motorRight2;
    motorRight1.setInverted(true);
    motorRight2.setInverted(true);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void moveLeft(double speedPercentage) {
    motorLeft1.set(ControlMode.PercentOutput, speedPercentage);
    motorLeft2.set(ControlMode.PercentOutput, speedPercentage);
  }

  public void moveRight(double speedPercentage) {
    motorRight1.set(ControlMode.PercentOutput, speedPercentage);
    motorRight2.set(ControlMode.PercentOutput, speedPercentage);
  }

  public void stop() {
    moveLeft(0.0);
    moveRight(0.0);
  }
}
