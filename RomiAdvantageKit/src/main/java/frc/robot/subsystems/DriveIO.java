// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

/** Drive subsystem hardware interface. */
public interface DriveIO {
  /** The set of loggable inputs for the drive subsystem. */
  public static class DriveIOInputs implements LoggableInputs {
    public double leftPosition = 0.0;
    public double leftVelocityRadPerSec = 0.0;
    public double leftAppliedVolts = 0.0;
    public double[] leftCurrentAmps = new double[] {};
    public double[] leftTempCelcius = new double[] {};

    public double rightPosition = 0.0;
    public double rightVelocityRadPerSec = 0.0;
    public double rightAppliedVolts = 0.0;
    public double[] rightCurrentAmps = new double[] {};
    public double[] rightTempCelcius = new double[] {};

    public boolean externalAvailable = false;
    public double externalLeftPositionRad = 0.0;
    public double externalLeftVelocityRadPerSec = 0.0;
    public double externalRightPositionRad = 0.0;
    public double externalRightVelocityRadPerSec = 0.0;

    public boolean gyroConnected = false;
    public double gyroYawPositionRad = 0.0;
    public double gyroYawVelocityRadPerSec = 0.0;
    public double gyroPitchPositionRad = 0.0;
    public double gyroRollPositionRad = 0.0;
    public double gyroZAccelMetersPerSec2 = 0.0;

    public void toLog(LogTable table) {
        table.put("LeftPosition", leftPosition);
        table.put("LeftVelocityRadPerSec", leftVelocityRadPerSec);
        table.put("LeftAppliedVolts", leftAppliedVolts);

        table.put("RightPosition", rightPosition);
        table.put("RightVelocityRadPerSec", rightVelocityRadPerSec);
        table.put("RightAppliedVolts", rightAppliedVolts);

        table.put("GyroConnected", gyroConnected);
        table.put("GyroYawPositionRad", gyroYawPositionRad);
        table.put("GyroYawVelocityRadPerSec", gyroYawVelocityRadPerSec);
    }

    public void fromLog(LogTable table) {
        leftPosition = table.get("LeftPositionRad", leftPosition);
        leftVelocityRadPerSec =
            table.get("LeftVelocityRadPerSec", leftVelocityRadPerSec);
        leftAppliedVolts = table.get("LeftAppliedVolts", leftAppliedVolts);

        rightPosition = table.get("RightPositionRad", rightPosition);
        rightVelocityRadPerSec =
            table.get("RightVelocityRadPerSec", rightVelocityRadPerSec);
        rightAppliedVolts =
            table.get("RightAppliedVolts", rightAppliedVolts);

        gyroConnected = table.get("GyroConnected", gyroConnected);
        gyroYawPositionRad =
            table.get("GyroYawPositionRad", gyroYawPositionRad);
        gyroYawVelocityRadPerSec =
            table.get("GyroYawVelocityRadPerSec", gyroYawVelocityRadPerSec);
    }
  }

  /** Updates the set of loggable inputs. */
  public default void updateInputs(DriveIOInputs inputs) {}

  /** Run open loop at the specified voltage. */
  public default void setVoltage(double leftVolts, double rightVolts) {}

  /** Run closed loop at the specified velocity. */
  public default void setVelocity(double leftVelocityRadPerSec,
      double rightVelocityRadPerSec, double leftFFVolts, double rightFFVolts) {}

  public default void arcadeDrive(double xaxisSpeed, double zaxisRotate) {}    

  public default void resetEncoders() {}

  /** Set velocity PID constants. */
  public default void configurePID(double kp, double ki, double kd) {}
}