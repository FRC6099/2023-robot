// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.AnalogPotentiometer;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.model.ArmAngles;
import frc.robot.model.ArmPosition;
import frc.robot.sim.PhysicsSim;

public class Arm extends SubsystemBase {
  // Inside robot -> Outside robot
  // ~475,500 -> 481,185
  // 475,500 -> 481,250 lower arm
  // 81,000 -> 93,000 upper arm
    // 18.588905112539916|-0.08043777812625565|101.49996093750146|24.58186523437501

  private static int TIMEOUT_MS = 30;
  private static int GAIN_PID = 0;
  private static int PID_LOOP_INDEX = 0;
  private static double LOWER_ARM_LENGTH = 36.0;           // INCHES
  private static double UPPER_ARM_LENGTH = 43.75;          // INCHES
  // private static double ARM_TICKS_PER_REVOLUTION = 128;    // ~148 Ticks per degree
  // private static double LOWER_MOTOR_REV_TO_ARM_REV = 125.0 / 1.0 * 60.0 / 18.0;   // 125:1, 60:18
  // private static double UPPER_MOTOR_REV_TO_ARM_REV = 100.0 / 1.0 * 60.0 / 18.0;   // 100:1, 60:18
  private static double LOWER_ARM_TICKS_PER_DEGREE = 4651 / 52.3;
  private static double UPPER_ARM_TICKS_PER_DEGREE = 4422 / 40.8;

  private final TalonSRX lowerArm = new WPI_TalonSRX(Constants.LOWER_ARM_MOTOR_CAN_ID);
  private final TalonSRX upperArm = new WPI_TalonSRX(Constants.UPPER_ARM_MOTOR_CAN_ID);
  private final DigitalInput lowerArmMaxLimit = new DigitalInput(Constants.LOWER_ARM_MAX_LIMIT_SWITCH_ID);
  private final DigitalInput lowerArmMinLimit = new DigitalInput(Constants.LOWER_ARM_MIN_LIMIT_SWITCH_ID);
  private final DigitalInput upperArmMinLimit = new DigitalInput(Constants.UPPER_ARM_LIMIT_SWITCH_ID);
  private final AnalogPotentiometer upperArmAngleSensor;

  private double lastSetXPosition = -1.0;
  private double lastSetYPosition = -1.0;
  private boolean isStopped = true;
  private long counter = 0;


  /** Creates a new Arm. */
  public Arm() {
    this.configureArm(lowerArm, Constants.START_LOWER_ARM_DEGREES, LOWER_ARM_TICKS_PER_DEGREE, true, 592, 400);
    this.configureArm(upperArm, Constants.START_UPPER_ARM_DEGREES, UPPER_ARM_TICKS_PER_DEGREE, true, 888, 800);
    AnalogInput sensor = new AnalogInput(Constants.ARM_MAX_ANGLE_POTENTIOMETER_ID);
    sensor.setAverageBits(2);
    upperArmAngleSensor = new AnalogPotentiometer(sensor, 1);
  }

  private void configureArm(TalonSRX arm, double angle, double turnRatio, boolean sensorPhase, double cruiseVelocity, double accel) {
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
		arm.setSensorPhase(sensorPhase);
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
		arm.configMotionCruiseVelocity(cruiseVelocity, TIMEOUT_MS);                  // SET THIS FOR MAX MOTOR SPEED
		arm.configMotionAcceleration(accel, TIMEOUT_MS);            // SET THIS FOR MAX MOTOR ACCELERATION

		/* Zero the sensor once on robot boot up */
    double angleTicks = angle * turnRatio;
		arm.setSelectedSensorPosition(angleTicks, PID_LOOP_INDEX, TIMEOUT_MS);
  }

  public void simulationInit() {
    System.out.print("Simulate Init for Arm");
    PhysicsSim.getInstance().addTalonSRX(lowerArm, 1.0, 8192, true);
    PhysicsSim.getInstance().addTalonSRX(upperArm, 1.0, 8192, true);
  }

  @Override
  public void simulationPeriodic() {
    PhysicsSim.getInstance().run();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    if (counter++ % 50 == 0) {
       ArmPosition position = getCurrentPosition();
       ArmAngles angles = getArmAngles(position);
       double rawLowerArmTicks = lowerArm.getSelectedSensorPosition(PID_LOOP_INDEX);
       double rawUpperArmTicks = upperArm.getSelectedSensorPosition(PID_LOOP_INDEX);
       SmartDashboard.putString("Arm Position", String.format("X: %.4f, Y: %.4f", position.getX(), position.getY()));
       SmartDashboard.putString("Arm Angles", String.format("Lower Angle: %.4f, Upper Angle: %.4f", angles.getLowerAngle(), angles.getUpperAngle()));
       SmartDashboard.putString("Arm Ticks", "Lower Ticks: " + rawLowerArmTicks + ", Upper Ticks: " + rawUpperArmTicks);
       SmartDashboard.putBoolean("Arm 1 Max Limit", lowerArmMaxLimit.get());
       SmartDashboard.putBoolean("Arm 1 Min Limit", lowerArmMinLimit.get());
       SmartDashboard.putBoolean("Arm 2 Min Limit", upperArmMinLimit.get());
       SmartDashboard.putNumber("Arm 2 Potentiometer", upperArmAngleSensor.get());
    }
  }

  public void moveLowerArm(double speed) {
    // System.out.println("Lower Min: " + lowerArmMinLimit.get() + "; Max: " + lowerArmMaxLimit.get() + "; Speed: " + speed);
    if ((!lowerArmMaxLimit.get() && speed < 0) || 
      (!lowerArmMinLimit.get() && speed > 0)
    ) {
      lowerArm.set(ControlMode.PercentOutput, 0.0);
    } else {
      lowerArm.set(ControlMode.PercentOutput, -speed);
      ArmPosition pos = getCurrentPosition();
      ArmAngles angle = getArmAngles(pos);
      System.out.println(pos.getX() + "|" + pos.getY() + "|" + angle.getLowerAngle() + "|" + angle.getUpperAngle());
    }
  }

  public void moveUpperArm(double speed) {
    // System.out.println("Upper Min: " + upperArmMinLimit.get() + "; Angle: " + upperArmAngleSensor.get() + "; Speed: " + speed);
    if ((!upperArmMinLimit.get() && speed < 0)// ||
      //(upperArmAngleSensor.get() * 180 / 5 >= 170.0 && speed > 0)
    ) {
      // System.out.println("Upper Arm Angle: " + upperArmAngleSensor.get());
      upperArm.set(ControlMode.PercentOutput, 0.0);
    } else {
      upperArm.set(ControlMode.PercentOutput, speed);
    }
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

  public boolean goToPosition(double x, double y) {
    return goToPosition(new ArmPosition(x, y));
  }

  public boolean goToPosition(ArmPosition targetPosition) {
    ArmPosition position = getCurrentPosition();
    double startX = position.getX();
    double startY = position.getY();
    double moveX = getLimitedValue(targetPosition.getX() - startX, 10.0);
    double moveY = getLimitedValue(targetPosition.getY() - startY, 10.0);
    // Check Boundaries & Adjust X, Y to min or max depending
    updatePositionToNext(moveX, moveY, position);

    // Calculate Arm Angles
    ArmAngles angles = getArmAngles(position);
    printStats(position, startX, startY, angles);

    // Set Angles
    setArmAngles(angles);
    return withinThreshold(startX, startY, targetPosition.getX(), targetPosition.getY());
  }

  private double getLimitedValue(double value, double limit) {
    double absLimit = Math.abs(limit);
    return Math.max(Math.min(value, absLimit), -absLimit);
  }
  
  private boolean withinThreshold(double currentX, double currentY, double targetX, double targetY) {
    double offsetX = Math.abs(targetX - currentX);
    double offsetY = Math.abs(targetY - currentY);
    return offsetX < 0.5 && offsetY < 0.5;
  }

  public void setArmAngles(ArmAngles angles) {
    setArmAngles(angles.getLowerAngle(), angles.getUpperAngle());
  }

  public void setArmAngles(double lowerDegrees, double upperDegrees) {
    double lowerTicks = lowerDegrees * LOWER_ARM_TICKS_PER_DEGREE;
    double upperTicks = upperDegrees * UPPER_ARM_TICKS_PER_DEGREE;
    lowerArm.set(ControlMode.MotionMagic, lowerTicks);
    upperArm.set(ControlMode.MotionMagic, upperTicks);
  }

  public void stop() {
    lowerArm.set(ControlMode.PercentOutput, 0.0);
    upperArm.set(ControlMode.PercentOutput, 0.0);
    isStopped = true;
  }

  private ArmPosition getCurrentPosition() {
    double lowerArmAngle = lowerArm.getSelectedSensorPosition(PID_LOOP_INDEX) / LOWER_ARM_TICKS_PER_DEGREE;
    double upperArmAngle = upperArm.getSelectedSensorPosition(PID_LOOP_INDEX) / UPPER_ARM_TICKS_PER_DEGREE;

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

  private void updatePositionToNext(double x, double y, ArmPosition currentPosition) {
    double startX = currentPosition.getX();
    double startY = currentPosition.getY();

    if (isStopped) {
      isStopped = false;
      lastSetXPosition = startX;
      lastSetYPosition = startY;
    }

    if (
      Math.abs(lastSetXPosition - startX) < 3.0 && 
      Math.abs(lastSetYPosition - startY) < 3.0
    ) {
      if (x == 0) {
        currentPosition.setX(lastSetXPosition);
      } else {
        currentPosition.addX(x);
        lastSetXPosition = currentPosition.getX();
      }
      if (y == 0) {
        currentPosition.setY(lastSetYPosition);
      } else {
        currentPosition.addY(y);
        lastSetYPosition = currentPosition.getY();
      }
    } else {
      currentPosition.setX(lastSetXPosition);
      currentPosition.setY(lastSetYPosition);
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
    if (Constants.ARM_LOGGING) {
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
}
