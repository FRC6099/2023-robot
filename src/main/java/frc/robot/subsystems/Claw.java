// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.model.ClawPosition;
import frc.robot.sim.PhysicsSim;

public class Claw extends SubsystemBase {

  private static int TIMEOUT_MS = 30;
  private static int GAIN_PID = 0;
  private static int PID_LOOP_INDEX = 0;
  private static double CLAW_TICKS_PER_REVOLUTION = 4096.0;
  private static double ENCODER_TO_CLAW_RATIO = 60.0 / 18.0;
  private static double MOTOR_TICKS_PER_DEGREE = CLAW_TICKS_PER_REVOLUTION / 360.0 * ENCODER_TO_CLAW_RATIO;

  private static final double OPENED_CLAW_ANGLE = 35.0;       // In Degrees (start position)
  private static final double CUBE_CLOSED_CLAW_ANGLE = 20.0;  // In Degrees
  private static final double CONE_CLOSED_CLAW_ANGLE = 15.0;  // In Degrees
  private final TalonSRX clawMotor = new WPI_TalonSRX(Constants.CLAW_MOTOR_CAN_ID);

  /** Creates a new Claw. */
  public Claw() {
    this.configureClaw();
  }

  private void configureClaw() {
    /* Factory default hardware to prevent unexpected behavior */
		clawMotor.configFactoryDefault();

		/* Configure Sensor Source for Pirmary PID */
		clawMotor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, TIMEOUT_MS);

		/* set deadband to super small 0.001 (0.1 %).
			The default deadband is 0.04 (4 %) */
    clawMotor.configNeutralDeadband(0.001, TIMEOUT_MS);

		/**
		 * Configure Talon SRX Output and Sensor direction accordingly Invert Motor to
		 * have green LEDs when driving Talon Forward / Requesting Postiive Output Phase
		 * sensor to have positive increment when driving Talon Forward (Green LED)
		 */
		clawMotor.setSensorPhase(false);
		clawMotor.setInverted(false);

		/* Set relevant frame periods to be at least as fast as periodic rate */
		clawMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, 10, TIMEOUT_MS);
		clawMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_10_MotionMagic, 10, TIMEOUT_MS);/* Set the peak and nominal outputs */
		
    clawMotor.configNominalOutputForward(0, TIMEOUT_MS);
		clawMotor.configNominalOutputReverse(0, TIMEOUT_MS);
		clawMotor.configPeakOutputForward(1, TIMEOUT_MS);                              // ALTERNATIVELY SET TO < 1.0 to control speed
		clawMotor.configPeakOutputReverse(-1, TIMEOUT_MS);                                         // ALTERNATIVELY SET TO > -1.0 to control speed

		/* Set Motion Magic gains in slot0 - see documentation */
		clawMotor.selectProfileSlot(GAIN_PID, PID_LOOP_INDEX);
		clawMotor.config_kF(GAIN_PID, 0.2, TIMEOUT_MS);
		clawMotor.config_kP(GAIN_PID, 0.2, TIMEOUT_MS);
		clawMotor.config_kI(GAIN_PID, 0.0, TIMEOUT_MS);
		clawMotor.config_kD(GAIN_PID, 0.0, TIMEOUT_MS);

		/* Set acceleration and vcruise velocity - see documentation */
		clawMotor.configMotionCruiseVelocity(4740 * 30, TIMEOUT_MS);                  // SET THIS FOR MAX MOTOR SPEED
		clawMotor.configMotionAcceleration(40, TIMEOUT_MS);            // SET THIS FOR MAX MOTOR ACCELERATION

		/* Zero the sensor once on robot boot up */
    double angleTicks = getSelectedStartingAngle() * MOTOR_TICKS_PER_DEGREE;
		clawMotor.setSelectedSensorPosition(angleTicks, PID_LOOP_INDEX, TIMEOUT_MS);
  }

  private double getSelectedStartingAngle() {
    switch(Constants.getStartingClawPosition()) {
      case OPEN:
        return OPENED_CLAW_ANGLE;
      case CUBE:
        return CUBE_CLOSED_CLAW_ANGLE;
      case CONE:
      case CLOSED:
      default:
        return CONE_CLOSED_CLAW_ANGLE;
    }
  }

  public void initSensorPosition() {
    /* Zero the sensor once on robot boot up */
    double angleTicks = getSelectedStartingAngle() * MOTOR_TICKS_PER_DEGREE;
		clawMotor.setSelectedSensorPosition(angleTicks, PID_LOOP_INDEX, TIMEOUT_MS);
  }
  
  public void simulationInit() {
    PhysicsSim.getInstance().addTalonSRX(clawMotor, 1.0, 8192, false);
  }

  @Override
  public void simulationPeriodic() {
    PhysicsSim.getInstance().run();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public boolean goToPosition(ClawPosition position) {
    if (Constants.isClawSpeedControlled()) {
      moveClaw(position);
    } else {
      return setAngle(position);
    }
    return false;
  }

  public void moveClaw(ClawPosition position) {
    switch(position) {
      case OPEN: 
      case CUBE:
        moveClaw(0.5);
        break;
      case CONE:
      case CLOSED:
        moveClaw(-0.5);
    }
  }

  public void moveClaw(double speed) {
    clawMotor.set(ControlMode.PercentOutput, speed);
  }

  private boolean setAngle(ClawPosition position) {
    switch(position) {
      case OPEN: 
        return setAngle(OPENED_CLAW_ANGLE);
      case CONE:
        return setAngle(CONE_CLOSED_CLAW_ANGLE);
      case CUBE:
      case CLOSED:
        return setAngle(CUBE_CLOSED_CLAW_ANGLE);
    }
    return true;
  }

  private boolean setAngle(double angle) {
    double motorTicks = angle * MOTOR_TICKS_PER_DEGREE;
    clawMotor.set(ControlMode.MotionMagic, motorTicks);
    double currentPosition = clawMotor.getSelectedSensorPosition();
    double offset = Math.abs(motorTicks - currentPosition);
    return offset <= 5.0;
  }

  public void stop() {
    clawMotor.set(ControlMode.PercentOutput, 0.0);
  }
}
