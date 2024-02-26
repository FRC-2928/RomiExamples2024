// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Units;
import frc.robot.subsystems.Drivetrain;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    public static final double countsPerRevolution = 1440.0;
    public static final Measure<Distance> wheelDiameter = Units.Meters.of(0.7);
    public static final Measure<Distance> wheelCircumference = wheelDiameter.times(Math.PI);
    public static final Measure<Distance> distancePerPulse = wheelDiameter.divide(countsPerRevolution);
    // public static final double kWheelDiameterInch = 2.75591; // 70 mm
}
