// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.List;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Unit;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotState;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    public enum RobotType {
        COMPETITION,
        DEVELOPMENT,
        REPLAY
    }
    public static final RobotType ROBOT_TYPE = RobotType.DEVELOPMENT;

  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }

  public static class FieldConstants{
    public static final Pose3d HUB_POSITION = new Pose3d(Units.inchesToMeters(158.6 + 47 / 2.0), 8.07 / 2, Units.inchesToMeters(72), new Rotation3d());; // TODO: FIX POSITION
  }

  public static class CALCULATIONS{
    public static <U extends Unit> boolean epsilonEquals(Measure<U> unit1, Measure<U> unit2, Measure<U> tolerance){
      return Math.abs(unit1.baseUnitMagnitude() - unit2.baseUnitMagnitude()) <= tolerance.baseUnitMagnitude();
    }
  }
}
