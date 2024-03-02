package frc.robot.sensors;

import edu.wpi.first.wpilibj.romi.RomiGyro;

public class GyroIORomiGyro implements GyroIO{
  // Set up the RomiGyro
  private final RomiGyro gyro = new RomiGyro();

  // Constructor
  public GyroIORomiGyro() {

  }

  @Override
  public void updateInputs(RomiGyroIOInputs inputs) {       
      inputs.heading = gyro.getAngleZ();
  }
}
