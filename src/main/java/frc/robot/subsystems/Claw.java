// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.model.ClawPosition;

public class Claw extends SubsystemBase {

  private static final double OPENED_CLAW_ANGLE = 0.0;
  private static final double MAX_CLOSED_CLAW_ANGLE = 100.0;
  private static final double CUBE_CLOSED_CLAW_ANGLE = 50.0;
  private static final double CONE_CLOSED_CLAW_ANGLE = 80.0;
  private final TalonSRX clawMotor = new WPI_TalonSRX(Constants.CLAW_MOTOR_CAN_ID);

  /** Creates a new Claw. */
  public Claw() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void openClaw() {
    setAngle(OPENED_CLAW_ANGLE);
  }

  public void closeClaw() {
    setAngle(MAX_CLOSED_CLAW_ANGLE);
  }

  public void goToPosition(ClawPosition position) {
    switch(position) {
      case OPEN: 
        setAngle(OPENED_CLAW_ANGLE);
        break;
      case CONE:
        setAngle(CONE_CLOSED_CLAW_ANGLE);
        break;
      case CUBE:
        setAngle(CUBE_CLOSED_CLAW_ANGLE);
        break;
    }
  }

  private void setAngle(double angle) {
    clawMotor.set(ControlMode.Position, angle);
  }
}
