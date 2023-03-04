// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Handbrake extends SubsystemBase {

  private static final double WAIT_PERIOD = 1.5;
  private static final String HANDBRAKE = "Handbrake";

  private final WPI_VictorSPX leftHandbrake = new WPI_VictorSPX(Constants.LEFT_HANDBRAKE_CAN_ID);
  private final WPI_VictorSPX rightHandbrake = new WPI_VictorSPX(Constants.RIGHT_HANDBRAKE_CAN_ID);

  private final Timer timer = new Timer();

  private boolean isEngaging = false;
  private boolean isEngaged = false;
  private boolean isReleasing = false;

  /** Creates a new Handbrake. */
  public Handbrake() {
    SmartDashboard.setDefaultBoolean(HANDBRAKE, false);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    if (isEngaging && !timer.hasElapsed(WAIT_PERIOD)) {
      setMotors(0.8);

    } else if (isEngaging) {
      stopMotors();
      timer.stop();
      isEngaged = true;
      SmartDashboard.putBoolean(HANDBRAKE, true);

    } else if (isReleasing && !timer.hasElapsed(WAIT_PERIOD)) {
      setMotors(-0.8);
      
    } else if (isReleasing) {
      stopMotors();
      timer.stop();
      isEngaged = false;
      SmartDashboard.putBoolean(HANDBRAKE, false);
    }
  }

  public void engage() {
    if (!isEngaging && !isEngaged) {
      isEngaging = true;
      isReleasing = false;
      timer.reset();
      timer.start();
    }
  }

  public void release() {
    if (!isReleasing && isEngaged) {
      isReleasing = true;
      isEngaging = false;
      timer.reset();
      timer.start();
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
