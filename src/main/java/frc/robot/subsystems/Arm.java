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
import frc.robot.model.ArmAngles;
import frc.robot.model.ArmPosition;
import frc.robot.sim.PhysicsSim;

public class Arm extends SubsystemBase {

  private static int TIMEOUT_MS = 30;
  private static int GAIN_PID = 0;
  private static int PID_LOOP_INDEX = 0;
  private static double LOWER_ARM_LENGTH = 36.0;           // INCHES
  private static double UPPER_ARM_LENGTH = 43.75;          // INCHES
  private static double ARM_TICKS_PER_REVOLUTION = 4096;

  private final TalonSRX lowerArm = new WPI_TalonSRX(0);
  private final TalonSRX upperArm = new WPI_TalonSRX(1);

  private double lastSetXPosition = -1.0;
  private double lastSetYPosition = -1.0;
  private boolean isStopped = true;


  /** Creates a new Arm. */
  public Arm() {
    this.configureArm(lowerArm, 110.0);
    this.configureArm(upperArm, 30.0);
  }

  private void configureArm(TalonSRX arm, double angle) {
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
    double angleTicks = angle * ARM_TICKS_PER_REVOLUTION / 360.0;
		arm.setSelectedSensorPosition(angleTicks, PID_LOOP_INDEX, TIMEOUT_MS);

    ArmPosition pos = getCurrentPosition();
    this.lastSetXPosition = pos.getX();
    this.lastSetYPosition = pos.getY();
  }

  public void simulationInit() {
    System.out.print("Simulate Init for Arm");
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
    ArmPosition position = getCurrentPosition();
    double startX = position.getX();
    double startY = position.getY();
    // Check Boundaries & Adjust X, Y to min or max depending
    updatePositionToNext(x, y, position);

    // Calculate Arm Angles
    ArmAngles angles = getArmAngles(position);
    printStats(position, startX, startY, angles);

    // Set Angles
    setArmAngles(angles);
  }

  public void setArmAngles(ArmAngles angles) {
    setArmAngles(angles.getLowerAngle(), angles.getUpperAngle());
  }

  public void setArmAngles(double lowerDegrees, double upperDegrees) {
    double lowerTicks = lowerDegrees * ARM_TICKS_PER_REVOLUTION / 360.0;
    double upperTicks = upperDegrees * ARM_TICKS_PER_REVOLUTION / 360.0;
    lowerArm.set(ControlMode.MotionMagic, lowerTicks);
    upperArm.set(ControlMode.MotionMagic, upperTicks);
  }

  public void stop() {
    lowerArm.set(ControlMode.PercentOutput, 0.0);
    upperArm.set(ControlMode.PercentOutput, 0.0);
    isStopped = true;
  }

  private ArmPosition getCurrentPosition() {
    double lowerArmAngle = lowerArm.getSelectedSensorPosition(PID_LOOP_INDEX) / ARM_TICKS_PER_REVOLUTION * 360;
    double upperArmAngle = upperArm.getSelectedSensorPosition(PID_LOOP_INDEX) / ARM_TICKS_PER_REVOLUTION * 360;

    // lengthC = (A^2 + B^2 - 2AB * cos(c))^1/2
    double lengthC = Math.sqrt(Math.pow(LOWER_ARM_LENGTH, 2) + Math.pow(UPPER_ARM_LENGTH, 2) - 2 * LOWER_ARM_LENGTH * UPPER_ARM_LENGTH * Math.cos(Math.toRadians(upperArmAngle)));
    // angleB = arccos((C^2 + A^2 - B^2) / (2AC))
    double angleB = Math.toDegrees(Math.acos((Math.pow(lengthC, 2) + Math.pow(LOWER_ARM_LENGTH, 2) - Math.pow(UPPER_ARM_LENGTH, 2)) / (2 * LOWER_ARM_LENGTH * lengthC)));
    double angleY = lowerArmAngle - angleB;
    double angleX = 90.0 - angleY;
    double lengthX = lengthC / Math.sin(Math.toRadians(90.0)) * Math.sin(Math.toRadians(angleX));
    double lengthY = lengthC / Math.sin(Math.toRadians(90.0)) * Math.sin(Math.toRadians(angleY));

    return new ArmPosition(lengthX, lengthY, new ArmAngles(lowerArmAngle, upperArmAngle));
  }

  private void updatePositionToNext(double x, double y, ArmPosition position) {
    double startX = position.getX();
    double startY = position.getY();

    if (isStopped) {
      isStopped = false;
      lastSetXPosition = startX;
      lastSetYPosition = startY;
    }

    if (Math.abs(lastSetXPosition - startX) < 3.0 && x!=0.0) {
      position.addX(x);
      lastSetXPosition = position.getX();
    } else {
      position.setX(lastSetXPosition);
    }
    if (Math.abs(lastSetYPosition - startY) < 3.0 && y!=0.0) {
      position.addY(y);
      lastSetYPosition = position.getY();
    } else {
      position.setY(lastSetYPosition);
    }
  }
  
  //                       U  L
  //                     U  c
  //                   U      L
  //             B  U           A
  //              U
  //           U              L
  //         U 
  //       *   a                
  //       * f  *              L
  // D = Y *      C   *      b  
  //       * e            d* r L
  //       * * * * * * * * * * *
  //                F = X
  private ArmAngles getArmAngles(ArmPosition position) {
    double lengthC = Math.sqrt(Math.pow(position.getX(), 2) + Math.pow(position.getY(), 2));
    // double angleA = Math.toDegrees(Math.acos((Math.pow(UPPER_ARM_LENGTH, 2) + Math.pow(lengthC, 2) - Math.pow(LOWER_ARM_LENGTH, 2)) / (2 * UPPER_ARM_LENGTH * lengthC)));
    double angleB = Math.toDegrees(Math.acos((Math.pow(LOWER_ARM_LENGTH, 2) + Math.pow(lengthC, 2) - Math.pow(UPPER_ARM_LENGTH, 2)) / (2 * LOWER_ARM_LENGTH * lengthC)));
    double angleC = Math.toDegrees(Math.acos((Math.pow(LOWER_ARM_LENGTH, 2) + Math.pow(UPPER_ARM_LENGTH, 2) - Math.pow(lengthC, 2)) / (2 * LOWER_ARM_LENGTH * UPPER_ARM_LENGTH)));
    double angleD = Math.toDegrees(Math.acos(position.getX() / lengthC));

    if (position.getY() < 0.0) {
      angleD = angleD * -1.0;
    }
    double angleR = angleB + angleD;

    // Angle A, B, C should equal 180
    return new ArmAngles(angleR, angleC);
  }

  private void printStats(ArmPosition pos, double startX, double startY, ArmAngles angles) {
    // System.out.println("Start X: " + (pos.getX()-x) + 
    //         "; Start Y: " + (pos.getY()-y) + 
    //         "; Next X: " + pos.getX() + 
    //         "; Next Y: " + pos.getY() + 
    //         "; Set X: " + lastSetXPosition + 
    //         "; Set Y: " + lastSetYPosition + 
    //         "; Start lower: " + pos.getArmAngles().getLowerAngle() + 
    //         "; Start upper: " + pos.getArmAngles().getUpperAngle() +
    //         "; Next lower: " + angles.getLowerAngle() + 
    //         "; Next upper: " + angles.getUpperAngle()
    //         );
    System.out.println(startX + 
            "|" + startY + 
            "|" + pos.getX() + 
            "|" + pos.getY() + 
            "|" + lastSetXPosition + 
            "|" + lastSetYPosition + 
            "|" + pos.getArmAngles().getLowerAngle() + 
            "|" + pos.getArmAngles().getUpperAngle() +
            "|" + angles.getLowerAngle() + 
            "|" + angles.getUpperAngle()
            );
  }
}
