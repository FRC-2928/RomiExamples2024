package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.romi.RomiGyro;
// import frc.robot.sensors.RomiGyro;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
// import edu.wpi.first.wpilibj.romi.RomiMotor;
import frc.robot.Constants;

public class DriveIORomi implements DriveIO {

  private final Spark leftMotor = new Spark(0);
  private final Spark rightMotor = new Spark(1);
  private final Encoder leftEncoder = new Encoder(4, 5);
  private final Encoder rightEncoder = new Encoder(6, 7);
  private final RomiGyro gyro = new RomiGyro();
  private PIDController leftPID =
      new PIDController(0.0, 0.0, 0.0, Constants.loopPeriodSecs);
  private PIDController rightPID =
      new PIDController(0.0, 0.0, 0.0, Constants.loopPeriodSecs);

  private boolean closedLoop = false;
  private double leftFFVolts = 0.0;
  private double rightFFVolts = 0.0;
  private double appliedVoltsLeft = 0.0;
  private double appliedVoltsRight = 0.0;

  // private final DifferentialDrive diffDrive = new DifferentialDrive(leftMotor, rightMotor);

  public DriveIORomi() {
    // Romi encoders have 1440 pulses per revolution
    this.leftEncoder.setDistancePerPulse((Math.PI * Constants.kWheelDiameterMeters) / Constants.kCountsPerRevolution);
    this.rightEncoder.setDistancePerPulse((Math.PI * Constants.kWheelDiameterMeters) / Constants.kCountsPerRevolution);
    // leftEncoder.setDistancePerPulse((2 * Math.PI) / 1440);
    // rightEncoder.setDistancePerPulse((2 * Math.PI) / 1440);
  }

  @Override
  public void updateInputs(DriveIOInputs inputs) {
    if (closedLoop) {
      double leftVolts = leftPID.calculate(leftEncoder.getRate()) + leftFFVolts;
      double rightVolts =
          rightPID.calculate(rightEncoder.getRate()) + rightFFVolts;
      appliedVoltsRight = rightVolts;
      appliedVoltsLeft = leftVolts;
      leftMotor.setVoltage(leftVolts);
      rightMotor.setVoltage(rightVolts * -1);
    }

    inputs.leftPosition = leftEncoder.getDistance();
    inputs.leftVelocity = leftEncoder.getRate();
    inputs.leftAppliedVolts = appliedVoltsLeft;

    inputs.rightPosition = rightEncoder.getDistance();
    inputs.rightVelocity = rightEncoder.getRate();
    inputs.rightAppliedVolts = appliedVoltsRight;

    inputs.gyroConnected = true;
    inputs.gyroYawPositionRad = Math.toRadians(gyro.getAngleZ());
    inputs.gyroYawVelocityRadPerSec = Math.toRadians(gyro.getRateZ());
  }

  @Override
  public void setVoltage(double leftVolts, double rightVolts) {
    closedLoop = false;
    appliedVoltsRight = rightVolts;
    appliedVoltsLeft = leftVolts;
    leftMotor.setVoltage(leftVolts * 1.1);
    rightMotor.setVoltage(rightVolts * -1);
  }

  @Override
  public void setVelocity(double leftVelocityRadPerSec,
      double rightVelocityRadPerSec, double leftFFVolts, double rightFFVolts) {
    closedLoop = true;
    leftPID.setSetpoint(leftVelocityRadPerSec);
    rightPID.setSetpoint(rightVelocityRadPerSec);
    this.leftFFVolts = leftFFVolts;
    this.rightFFVolts = rightFFVolts;
  }

  // @Override
  // public void arcadeDrive(double xaxisSpeed, double zaxisRotate) {
  //   this.diffDrive.arcadeDrive(xaxisSpeed, zaxisRotate);
  // }

  @Override
  public void resetEncoders() {
    this.leftEncoder.reset();
    this.rightEncoder.reset();
  }

  @Override
  public void configurePID(double kp, double ki, double kd) {
    leftPID.setP(kp);
    leftPID.setI(ki);
    leftPID.setD(kd);
    rightPID.setP(kp);
    rightPID.setI(ki);
    rightPID.setD(kd);
  }
}