// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Handbrake extends SubsystemBase {

  private static final int WAIT_PERIOD = 10;

  private final WPI_VictorSPX leftHandbrake = new WPI_VictorSPX(Constants.LEFT_HANDBRAKE_CAN_ID);
  private final WPI_VictorSPX rightHandbrake = new WPI_VictorSPX(Constants.RIGHT_HANDBRAKE_CAN_ID);

  private int timeout = 0;

  /** Creates a new Handbrake. */
  public Handbrake() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void engage() {
    if (timeout < 0) {
      timeout = 0;
    }

    if (timeout++ < WAIT_PERIOD) {
      setMotors(0.2);
    } else {
      stopMotors();
    }
  }

  public void release() {
    if (timeout > 5) {
      timeout = 5;
    }

    if (timeout-- > 0) {
      setMotors(-0.2);
    } else {
      stopMotors();
    }
  }

  private void setMotors(double speed) {
    leftHandbrake.set(ControlMode.PercentOutput, speed);
    rightHandbrake.set(ControlMode.PercentOutput, speed);
  }

  private void stopMotors() {
    setMotors(0.0);
  }
}
