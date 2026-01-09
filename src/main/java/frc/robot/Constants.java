// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;

public final class Constants {
  public static final class DriveConstants {
    // Allowed max speeds (not absolute hardware max)
    public static final double kMaxSpeedMetersPerSecond = 4.8;
    public static final double kMaxAngularSpeed = 2 * Math.PI; // rad/s

    // Chassis configuration
    public static final double kTrackWidth = Units.inchesToMeters(26.5);
    public static final double kWheelBase  = Units.inchesToMeters(26.5);

    public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
        new Translation2d( kWheelBase / 2,  kTrackWidth / 2),
        new Translation2d( kWheelBase / 2, -kTrackWidth / 2),
        new Translation2d(-kWheelBase / 2,  kTrackWidth / 2),
        new Translation2d(-kWheelBase / 2, -kTrackWidth / 2));

    // Module angle offsets (radians)
    public static final double kFrontLeftChassisAngularOffset  = -Math.PI / 2;
    public static final double kFrontRightChassisAngularOffset = 0;
    public static final double kBackLeftChassisAngularOffset   = Math.PI;
    public static final double kBackRightChassisAngularOffset  = Math.PI / 2;

    // SPARK MAX CAN IDs
    public static final int kFrontLeftDrivingCanId  = 11;
    public static final int kRearLeftDrivingCanId   = 13;
    public static final int kFrontRightDrivingCanId = 15;
    public static final int kRearRightDrivingCanId  = 17;

    public static final int kFrontLeftTurningCanId  = 10;
    public static final int kRearLeftTurningCanId   = 12;
    public static final int kFrontRightTurningCanId = 14;
    public static final int kRearRightTurningCanId  = 16;

    public static final boolean kGyroReversed = false;
  }

  public static final class ModuleConstants {
    public static final int kDrivingMotorPinionTeeth = 14;
    public static final double kDrivingMotorFreeSpeedRps = NeoMotorConstants.kFreeSpeedRpm / 60.0;
    public static final double kWheelDiameterMeters = 0.0762;
    public static final double kWheelCircumferenceMeters = kWheelDiameterMeters * Math.PI;
    // 45T wheel bevel, 22T first-stage spur, 15T bevel pinion
    public static final double kDrivingMotorReduction =
        (45.0 * 22.0) / (kDrivingMotorPinionTeeth * 15.0);
    public static final double kDriveWheelFreeSpeedRps =
        (kDrivingMotorFreeSpeedRps * kWheelCircumferenceMeters) / kDrivingMotorReduction;
  }

  public static final class OIConstants {
    public static final int kDriverControllerPort = 0;
    public static final double kDriveDeadband = 0.05;
  }

  public static final class AutoConstants {
    public static final double kMaxSpeedMetersPerSecond = 3.0;
    public static final double kMaxAccelerationMetersPerSecondSquared = 3.0;
    public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
    public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;

    public static final double kPXController = 1.0;
    public static final double kPYController = 1.0;
    public static final double kPThetaController = 1.0;

    public static final TrapezoidProfile.Constraints kThetaControllerConstraints =
        new TrapezoidProfile.Constraints(
            kMaxAngularSpeedRadiansPerSecond,
            kMaxAngularSpeedRadiansPerSecondSquared);
  }

  public static final class NeoMotorConstants {
    public static final double kFreeSpeedRpm = 5676.0;
  }

  public static final class ExampleConstants {
    // CAN ID of the Spark Max controlling test motor
    public static final int kExampleMotorCanId = 20;

    // PID gains for position control (tune on robot)
    public static final double kP = 1.0;
    public static final double kI = 0.0;
    public static final double kD = 0.0;

    // Optional gear ratio (output rotations per motor rotation)
    public static final double kPositionConversionFactor = 1.0;

    // Target position (in rotations)
    public static final double kTargetRotations = 10.0;

    // Limits for safety
    public static final double kMinRotations = -100.0;
    public static final double kMaxRotations = 100.0;
  }
}
