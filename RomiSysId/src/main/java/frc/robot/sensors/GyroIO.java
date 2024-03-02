package frc.robot.sensors;

import edu.wpi.first.wpilibj.romi.RomiGyro;

import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

/**
 * Connects the software to the hardware and directly receives data from the gyroscope.
 */
public interface GyroIO {

    /** Contains all of the input data received from hardware. */
    public static class RomiGyroIOInputs implements LoggableInputs {

      public double heading = 0.0;
    
      public void fromLog(LogTable table) {
        heading = table.get("Heading", heading);
      }  

      public void toLog(LogTable table) {
        table.put("Heading", heading);
      }  
    }

    /**
     * Reads information from sources (hardware or simulation) and updates the inputs object.
     *
     * @param inputs Contains the defaults for the input values listed above.
     */
    default void updateInputs(RomiGyroIOInputs inputs) {}

    

    /**
     * Holds data that can be read from the corresponding gyroscope IO implementation.
     */
    // @AutoLog
    // class RomiGyroIOInputs {
    //   public double heading;
    // }
}