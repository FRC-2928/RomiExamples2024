// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.BuiltInAccelerometer;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import frc.robot.Constants;
import frc.robot.sensors.RomiGyro;
import frc.robot.subsystems.DriveIO.DriveIOInputs;
import frc.robot.sensors.GyroIORomiGyro;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import org.littletonrobotics.junction.Logger;

public class Drivetrain extends SubsystemBase {

  // The Romi has the left and right motors set to
  // PWM channels 0 and 1 respectively
  // private final Spark leftMotor = new Spark(0);
  // private final Spark rightMotor = new Spark(1);

  // // The Romi has onboard encoders that are hardcoded
  // // to use DIO pins 4/5 and 6/7 for the left and right
  // private final Encoder leftEncoder = new Encoder(4, 5);
  // private final Encoder rightEncoder = new Encoder(6, 7);

  // Set up the differential drive controller
  // private final DifferentialDrive diffDrive = new DifferentialDrive(this.leftMotor, this.rightMotor);

  // Set up the RomiGyro
  private final RomiGyro gyro = new RomiGyro();

  // Set up the BuiltInAccelerometer
  private final BuiltInAccelerometer accelerometer = new BuiltInAccelerometer();

  private final DriveIO io;
  private final DriveIOInputs inputs = new DriveIOInputs();

  private double lastLeftDistanceMeters = 0.0;
  private double lastRightDistanceMeters = 0.0;
  private Rotation2d lastGyroRotation = new Rotation2d();

  /** Creates a new Drivetrain. */
  public Drivetrain(DriveIO io) {

    this.io = io;

    // maxVelocityMetersPerSec = 0.6;
    // wheelRadiusMeters = 0.035;
    // trackWidthMeters = 0.281092;
    // leftModel = new SimpleMotorFeedforward(0.27034, 0.64546, 0.021935);
    // rightModel = new SimpleMotorFeedforward(0.48548, 0.37427, 0.07421);
    // kP.setDefault(0.25);
    // kD.setDefault(0.001);

    // We need to invert one side of the drivetrain so that positive voltages
    // result in both sides moving forward. Depending on how your robot's
    // gearbox is constructed, you might have to invert the left side instead.
    // this.rightMotor.setInverted(true);

    // // Use inches as unit for encoder distances
    // this.leftEncoder.setDistancePerPulse((Math.PI * Constants.kWheelDiameterMeters) / Constants.kCountsPerRevolution);
    // this.rightEncoder.setDistancePerPulse((Math.PI * Constants.kWheelDiameterMeters) / Constants.kCountsPerRevolution);
    resetEncoders();
  }

  public void arcadeDrive(double xaxisSpeed, double zaxisRotate) {
    io.arcadeDrive(xaxisSpeed, zaxisRotate);
  }

  public void resetEncoders() {
    io.resetEncoders();
  }

  // public int getLeftEncoderCount() {
  //   return this.leftEncoder.get();
  // }

  // public int getRightEncoderCount() {
  //   return this.rightEncoder.get();
  // }

  public double getLeftDistanceMeters() {
    return inputs.leftPosition;
  }

  public double getRightDistanceMeters() {
    return inputs.rightPosition;
  }

  public double getAverageDistanceMeters() {
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
    io.updateInputs(inputs);
    Logger.processInputs("Drive", inputs);

    // Update odometry
    Rotation2d currentGyroRotation =
        new Rotation2d(inputs.gyroYawPositionRad * -1);
    // double leftDistanceMetersDelta =
    //     getLeftDistanceMeters() - lastLeftDistanceMeters;
    // double rightDistanceMetersDelta =
    //     getRightDistanceMeters() - lastRightDistanceMeters;

    // lastLeftDistanceMeters = getLeftDistanceMeters();
    // lastRightDistanceMeters = getRightDistanceMeters();
    lastGyroRotation = currentGyroRotation;
  }
}
