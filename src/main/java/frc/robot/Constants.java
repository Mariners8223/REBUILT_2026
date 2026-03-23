// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Unit;

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
    public static final Pose2d HUB_POSITION = new Pose2d(4.611,4.046,new Rotation2d());
    public static final Pose2d PRE_PRE_TOWER_POSITION = new Pose2d(15.66,5.42, Rotation2d.fromDegrees(54.7));
    public static final Pose2d PRE_TOWER_POSITION = new Pose2d(15.86,5.15, Rotation2d.kCCW_90deg);
    public static final Pose2d TOWER_POSITION = new Pose2d(15.72,5.15, Rotation2d.kCCW_90deg);
    public static final Pose2d INNER_TOWER_POSITION = new Pose2d(15.72,4.98, Rotation2d.kCCW_90deg);

    public static final double RED_ALLIANCE_ZONE_X = 12;
    public static final double BLUE_ALLIANCE_ZONE_X = 4;
  }

  public static class CALCULATIONS{
    public static <U extends Unit> boolean epsilonEquals(Measure<U> unit1, Measure<U> unit2, Measure<U> tolerance){
      return Math.abs(unit1.baseUnitMagnitude() - unit2.baseUnitMagnitude()) <= tolerance.baseUnitMagnitude();
    }
  }
    public static final class autoConstats{

    static double maxRange = 5; //TODO: find this
    static Pose2d hub = new Pose2d(4.611,4.046,new Rotation2d());
    static Pose2d topShoot = new Pose2d(3.045,4.496,new Rotation2d());
    static Pose2d bottomShoot = new Pose2d(3.045,3.495,new Rotation2d());


    public static final class TrenchLocations{
      Pose2d upRightTrench = new Pose2d(5.825,7.341,new Rotation2d());
      Pose2d upLeftTrench = new Pose2d(3.573,7.341,new Rotation2d());
      Pose2d downRightTrench = new Pose2d(5.760,0.652,new Rotation2d());
      Pose2d downLeftTrench = new Pose2d(3.560,0.652,new Rotation2d());
    }
  }
}
