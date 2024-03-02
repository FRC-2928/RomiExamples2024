// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.MutableMeasure.mutable;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.MutableMeasure;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj.BuiltInAccelerometer;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.romi.RomiGyro;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;

public class Drivetrain extends SubsystemBase {

  // The Romi has the left and right motors set to
  // PWM channels 0 and 1 respectively
  private final Spark leftMotor = new Spark(0);
  private final Spark rightMotor = new Spark(1);

  // The Romi has onboard encoders that are hardcoded
  // to use DIO pins 4/5 and 6/7 for the left and right
  private final Encoder leftEncoder = new Encoder(4, 5);
  private final Encoder rightEncoder = new Encoder(6, 7);

  // Set up the differential drive controller
  private final DifferentialDrive diffDrive =
      new DifferentialDrive(leftMotor::set, rightMotor::set);

  // Set up the RomiGyro
  private final RomiGyro gyro = new RomiGyro();

  // Set up the BuiltInAccelerometer
  private final BuiltInAccelerometer accelerometer = new BuiltInAccelerometer();

  // Mutable holder for unit-safe voltage values, persisted to avoid reallocation.
  private final MutableMeasure<Voltage> appliedVoltage = mutable(Volts.of(0));
  // Mutable holder for unit-safe linear distance values, persisted to avoid reallocation.
  private final MutableMeasure<Distance> distance = mutable(Meters.of(0));
  // Mutable holder for unit-safe linear velocity values, persisted to avoid reallocation.
  private final MutableMeasure<Velocity<Distance>> velocity = mutable(MetersPerSecond.of(0));

  // Create a new SysId routine for characterizing the drive.
  private final SysIdRoutine sysIdRoutine =
      new SysIdRoutine(
          // Empty config defaults to 1 volt/second ramp rate and 7 volt step voltage.
          new SysIdRoutine.Config(),
          new SysIdRoutine.Mechanism(
              // Tell SysId how to plumb the driving voltage to the motors.
              (Measure<Voltage> volts) -> {
                this.leftMotor.setVoltage(volts.in(Volts));
                this.rightMotor.setVoltage(volts.in(Volts));
              },
              // Tell SysId how to record a frame of data for each motor on the mechanism being
              // characterized.
              log -> {
                // Record a frame for the left motors.  Since these share an encoder, we consider
                // the entire group to be one motor.
                log.motor("drive-left")
                    .voltage(this.appliedVoltage.mut_replace(
                            this.leftMotor.get() * RobotController.getBatteryVoltage(), Volts))
                    .linearPosition(this.distance.mut_replace(this.leftEncoder.getDistance(), Meters))
                    .linearVelocity(this.velocity.mut_replace(this.leftEncoder.getRate(), MetersPerSecond));
                // Record a frame for the right motors.  Since these share an encoder, we consider
                // the entire group to be one motor.
                log.motor("drive-right")
                    .voltage(this.appliedVoltage.mut_replace(
                            this.rightMotor.get() * RobotController.getBatteryVoltage(), Volts))
                    .linearPosition(this.distance.mut_replace(this.rightEncoder.getDistance(), Meters))
                    .linearVelocity(this.velocity.mut_replace(this.rightEncoder.getRate(), MetersPerSecond));
              },
              // Tell SysId to make generated commands require this subsystem, suffix test state in
              // WPILog with this subsystem's name ("drive")
              this));

  /** Creates a new Drivetrain. */
  public Drivetrain() {
    SendableRegistry.addChild(this.diffDrive, this.leftMotor);
    SendableRegistry.addChild(this.diffDrive, this.rightMotor);

    // We need to invert one side of the drivetrain so that positive voltages
    // result in both sides moving forward. Depending on how your robot's
    // gearbox is constructed, you might have to invert the left side instead.
    this.rightMotor.setInverted(true);

    // Use inches as unit for encoder distances
    this.leftEncoder.setDistancePerPulse((Math.PI * Constants.kWheelDiameterMeters) / Constants.kCountsPerRevolution);
    this.rightEncoder.setDistancePerPulse((Math.PI * Constants.kWheelDiameterMeters) / Constants.kCountsPerRevolution);
    resetEncoders();
  }

  /** Returns a command to run a quasistatic test in the specified direction. */
	public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
		return sysIdRoutine.quasistatic(direction);
	}

	/** Returns a command to run a dynamic test in the specified direction. */
	public Command sysIdDynamic(SysIdRoutine.Direction direction) {
		return sysIdRoutine.dynamic(direction);
	}

  public void arcadeDrive(double xaxisSpeed, double zaxisRotate) {
    this.diffDrive.arcadeDrive(xaxisSpeed, zaxisRotate);
  }

  public void voltageDrive(Measure<Voltage> leftVolts, Measure<Voltage> rightVolts) {
    this.voltageDrive(leftVolts.baseUnitMagnitude(), rightVolts.baseUnitMagnitude());
  }

  public void voltageDrive(double leftVolts, double rightVolts) {
    this.leftMotor.setVoltage(leftVolts);
    this.rightMotor.setVoltage(rightVolts); 
    this.diffDrive.feed();
  }

  public void resetEncoders() {
    this.leftEncoder.reset();
    this.rightEncoder.reset();
  }

  public int getLeftEncoderCount() {
    return this.leftEncoder.get();
  }

  public int getRightEncoderCount() {
    return this.rightEncoder.get();
  }

  public double getLeftDistanceMeters() {
    return this.leftEncoder.getDistance();
  }

  public double getRightDistanceMeters() {
    return this.rightEncoder.getDistance();
  }

  public double getAverageDistanceInch() {
    return (getLeftDistanceMeters() + getRightDistanceMeters()) / 2.0;
  }

  /**
   * The acceleration in the X-axis.
   *
   * @return The acceleration of the Romi along the X-axis in Gs
   */
  public double getAccelX() {
    return this.accelerometer.getX();
  }

  /**
   * The acceleration in the Y-axis.
   *
   * @return The acceleration of the Romi along the Y-axis in Gs
   */
  public double getAccelY() {
    return this.accelerometer.getY();
  }

  /**
   * The acceleration in the Z-axis.
   *
   * @return The acceleration of the Romi along the Z-axis in Gs
   */
  public double getAccelZ() {
    return this.accelerometer.getZ();
  }

  /**
   * Current angle of the Romi around the X-axis.
   *
   * @return The current angle of the Romi in degrees
   */
  public double getGyroAngleX() {
    return this.gyro.getAngleX();
  }

  /**
   * Current angle of the Romi around the Y-axis.
   *
   * @return The current angle of the Romi in degrees
   */
  public double getGyroAngleY() {
    return this.gyro.getAngleY();
  }

  /**
   * Current angle of the Romi around the Z-axis.
   *
   * @return The current angle of the Romi in degrees
   */
  public double getGyroAngleZ() {
    return this.gyro.getAngleZ();
  }

  /** Reset the gyro. */
  public void resetGyro() {
    this.gyro.reset();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
