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
import frc.robot.sim.PhysicsSim;

public class Arm extends SubsystemBase {

  private static int TIMEOUT_MS = 30;
  private static int GAIN_PID = 0;
  private static int PID_LOOP_INDEX = 0;

  private final TalonSRX lowerArm = new WPI_TalonSRX(0);
  private final TalonSRX upperArm = new WPI_TalonSRX(1);

  /** Creates a new Arm. */
  public Arm() {
    this.configureArm(lowerArm);
    this.configureArm(upperArm);
  }

  private void configureArm(TalonSRX arm) {
    /* Factory default hardware to prevent unexpected behavior */
		arm.configFactoryDefault();

		/* Configure Sensor Source for Pirmary PID */
		arm.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, TIMEOUT_MS);

		/* set deadband to super small 0.001 (0.1 %).
			The default deadband is 0.04 (4 %) */
    arm.configNeutralDeadband(0.001, TIMEOUT_MS);

		/**
		 * Configure Talon SRX Output and Sensor direction accordingly Invert Motor to
		 * have green LEDs when driving Talon Forward / Requesting Postiive Output Phase
		 * sensor to have positive increment when driving Talon Forward (Green LED)
		 */
		arm.setSensorPhase(false);
		arm.setInverted(false);

		/* Set relevant frame periods to be at least as fast as periodic rate */
		arm.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, 10, TIMEOUT_MS);
		arm.setStatusFramePeriod(StatusFrameEnhanced.Status_10_MotionMagic, 10, TIMEOUT_MS);

		/* Set the peak and nominal outputs */
		arm.configNominalOutputForward(0, TIMEOUT_MS);
		arm.configNominalOutputReverse(0, TIMEOUT_MS);
		arm.configPeakOutputForward(1, TIMEOUT_MS);                              // ALTERNATIVELY SET TO < 1.0 to control speed
		arm.configPeakOutputReverse(-1, TIMEOUT_MS);                                         // ALTERNATIVELY SET TO > -1.0 to control speed

		/* Set Motion Magic gains in slot0 - see documentation */
		arm.selectProfileSlot(GAIN_PID, PID_LOOP_INDEX);
		arm.config_kF(GAIN_PID, 0.2, TIMEOUT_MS);
		arm.config_kP(GAIN_PID, 0.2, TIMEOUT_MS);
		arm.config_kI(GAIN_PID, 0.0, TIMEOUT_MS);
		arm.config_kD(GAIN_PID, 0.0, TIMEOUT_MS);

		/* Set acceleration and vcruise velocity - see documentation */
		arm.configMotionCruiseVelocity(7, TIMEOUT_MS);                  // SET THIS FOR MAX MOTOR SPEED
		arm.configMotionAcceleration(300, TIMEOUT_MS);            // SET THIS FOR MAX MOTOR ACCELERATION

		/* Zero the sensor once on robot boot up */
		arm.setSelectedSensorPosition(0, PID_LOOP_INDEX, TIMEOUT_MS);
  }

  public void simulationInit() {
    PhysicsSim.getInstance().addTalonSRX(lowerArm, 1.0, 8192, false);
    PhysicsSim.getInstance().addTalonSRX(upperArm, 1.0, 8192, false);
  }

  @Override
  public void simulationPeriodic() {
    PhysicsSim.getInstance().run();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void addPosition(double x, double y) {
    // GET current X and Y
    // Check Boundaries & Adjust X, Y to min or max depending
    // Calculate Arm Angles
    // Set Angles
  }

  public void stop() {
    lowerArm.set(ControlMode.PercentOutput, 0.0);
    upperArm.set(ControlMode.PercentOutput, 0.0);
  }
}
