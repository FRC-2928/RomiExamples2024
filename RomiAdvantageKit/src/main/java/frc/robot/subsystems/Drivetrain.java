// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.BuiltInAccelerometer;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.romi.RomiGyro;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.subsystems.DriveIO.DriveIOInputs;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Drivetrain extends SubsystemBase {

  // Set up the RomiGyro
  private final RomiGyro m_gyro = new RomiGyro();

  // Set up the BuiltInAccelerometer
  private final BuiltInAccelerometer m_accelerometer = new BuiltInAccelerometer();

  private final DriveIO io;
  private final DriveIOInputs inputs = new DriveIOInputs();

  // Odometry class for tracking robot pose
  private final DifferentialDriveOdometry m_odometry;

  // Show a field diagram for tracking odometry
  private final Field2d m_field2d = new Field2d();

  // -----------------------------------------------------------
  // Initialization
  // -----------------------------------------------------------

  /** Creates a new Drivetrain. */
  public Drivetrain(DriveIO io) {
    // Assign the hardware IO layer
    this.io = io;

    resetEncoders();

    // Setup Odometry and Field2d view    
    m_field2d.setRobotPose(Constants.initialPose);
    m_odometry = new DifferentialDriveOdometry(getHeading(), 
                   getLeftDistanceMeters(), getRightDistanceMeters(), 
                   Constants.initialPose);

    // Display the field on SmartDashboard and Simulator               
    SmartDashboard.putData("field", m_field2d);
  }

  // -----------------------------------------------------------
  // Control Input
  // -----------------------------------------------------------

  public void arcadeDrive(double xSpeed, double zRotation) {
    var speeds = DifferentialDrive.arcadeDriveIK(xSpeed, zRotation, true);
    io.setVoltage(speeds.left * 12.0, speeds.right * 12.0);
  }

  /**
   * Resets the odometry to the specified pose
   * @param pose The pose to which to set the odometry
   */
  public void resetOdometry(Pose2d pose) {
    resetEncoders();
    resetGyro();
    m_odometry.resetPosition(getHeading(),
                            getLeftDistanceMeters(), 
                            getRightDistanceMeters(), 
                            pose);
  }

  public void resetEncoders() {
    io.resetEncoders();
  }

  /** Reset the gyro. */
  public void resetGyro() {
    m_gyro.reset();
  }

  // -----------------------------------------------------------
  // System State
  // -----------------------------------------------------------
  @AutoLogOutput
  public double getLeftDistanceMeters() {
    return inputs.leftPosition;
  }

  @AutoLogOutput
  public double getRightDistanceMeters() {
    return inputs.rightPosition;
  }

  @AutoLogOutput
  public double getAverageDistanceMeters() {
    return (getLeftDistanceMeters() + getRightDistanceMeters()) / 2.0;
  }

  /** Returns the velocity of the left wheels in meters/second. */
  @AutoLogOutput
  public double getLeftVelocityMeters() {
    return inputs.leftVelocity;
  }

  /** Returns the velocity of the right wheels in meters/second. */
  @AutoLogOutput
  public double getRightVelocityMeters() {
    return inputs.rightVelocity;
  }


  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    // Convert to wheel speeds
    return new DifferentialDriveWheelSpeeds(getLeftVelocityMeters(), 
                                            getRightVelocityMeters());
  }

  /**
   * The acceleration in the X-axis.
   *
   * @return The acceleration of the Romi along the X-axis in Gs
   */
  public double getAccelX() {
    return m_accelerometer.getX();
  }

  /**
   * The acceleration in the Y-axis.
   *
   * @return The acceleration of the Romi along the Y-axis in Gs
   */
  public double getAccelY() {
    return m_accelerometer.getY();
  }

  /**
   * The acceleration in the Z-axis.
   *
   * @return The acceleration of the Romi along the Z-axis in Gs
   */
  public double getAccelZ() {
    return m_accelerometer.getZ();
  }

  /**
   * Current angle of the Romi around the X-axis.
   *
   * @return The current angle of the Romi in degrees
   */
  public double getGyroAngleX() {
    return m_gyro.getAngleX();
  }

  /**
   * Current angle of the Romi around the Y-axis.
   *
   * @return The current angle of the Romi in degrees
   */
  public double getGyroAngleY() {
    return m_gyro.getAngleY();
  }

  /**
   * Current angle of the Romi around the Z-axis.
   *
   * @return The current angle of the Romi in degrees
   */
  public double getGyroAngleZ() {
    return m_gyro.getAngleZ();
  }

  /**
   * Current heading of the Romi around the Z-axis.
   *
   * @return The current Rotation2d heading of the Romi
   */
  public Rotation2d getHeading() {
    return new Rotation2d(getGyroAngleZ() * (Math.PI/180));
  }

  /** Returns the current odometry pose in meters. */
  @AutoLogOutput
  public Pose2d getPose() {
    return m_odometry.getPoseMeters();
  }

  // -----------------------------------------------------------
  // Process Logic
  // -----------------------------------------------------------
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    io.updateInputs(inputs);
    Logger.processInputs("Drive", inputs);

    // Update odometry
    m_odometry.update(getHeading(), 
              getLeftDistanceMeters(), 
              getRightDistanceMeters());

    publishTelemetry();          
  }

  public void publishTelemetry() {
  
    SmartDashboard.putNumber("Left wheel position", getLeftDistanceMeters());
    SmartDashboard.putNumber("Right wheel position", getRightDistanceMeters());
    SmartDashboard.putNumber("Average Distance", getAverageDistanceMeters());
    SmartDashboard.putNumber("Heading", getHeading().getDegrees());
    SmartDashboard.putNumber("Gyro Z", getGyroAngleZ());

    m_field2d.setRobotPose(m_odometry.getPoseMeters()); 
  }
}
