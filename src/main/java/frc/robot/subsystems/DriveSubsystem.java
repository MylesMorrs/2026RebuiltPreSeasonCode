package frc.robot.subsystems;

import edu.wpi.first.hal.FRCNetComm.tInstances;
import edu.wpi.first.hal.FRCNetComm.tResourceType;
import edu.wpi.first.hal.HAL;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

import edu.wpi.first.wpilibj.ADIS16470_IMU;
import edu.wpi.first.wpilibj.ADIS16470_IMU.IMUAxis;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase; 
import edu.wpi.first.wpilibj.Timer;

import edu.wpi.first.wpilibj.simulation.ADIS16470_IMUSim;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.DriveConstants;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

public class DriveSubsystem extends SubsystemBase {
  // MAXSwerveModules
  private final MAXSwerveModule m_frontLeft = new MAXSwerveModule(
      DriveConstants.kFrontLeftDrivingCanId,
      DriveConstants.kFrontLeftTurningCanId,
      DriveConstants.kFrontLeftChassisAngularOffset);

  private final MAXSwerveModule m_frontRight = new MAXSwerveModule(
      DriveConstants.kFrontRightDrivingCanId,
      DriveConstants.kFrontRightTurningCanId,
      DriveConstants.kFrontRightChassisAngularOffset);

  private final MAXSwerveModule m_rearLeft = new MAXSwerveModule(
      DriveConstants.kRearLeftDrivingCanId,
      DriveConstants.kRearLeftTurningCanId,
      DriveConstants.kBackLeftChassisAngularOffset);

  private final MAXSwerveModule m_rearRight = new MAXSwerveModule(
      DriveConstants.kRearRightDrivingCanId,
      DriveConstants.kRearRightTurningCanId,
      DriveConstants.kBackRightChassisAngularOffset);

  // Gyro
  private final ADIS16470_IMU m_gyro = new ADIS16470_IMU();

  // Odometry
  private final SwerveDriveOdometry m_odometry = new SwerveDriveOdometry(
      DriveConstants.kDriveKinematics,
      Rotation2d.fromDegrees(m_gyro.getAngle(IMUAxis.kZ)),
      new SwerveModulePosition[] {
          m_frontLeft.getPosition(),
          m_frontRight.getPosition(),
          m_rearLeft.getPosition(),
          m_rearRight.getPosition()
      });

  // ---- Simulation helpers ----
  private final Field2d m_field = new Field2d();
  private final ADIS16470_IMUSim m_gyroSim = new ADIS16470_IMUSim(m_gyro);
  private double m_lastTime = Timer.getFPGATimestamp();
  public Field2d getField() { return m_field; }

  // Cache last commanded states so sim has “feedback”
  private SwerveModuleState[] m_lastSetStates = new SwerveModuleState[] {
      new SwerveModuleState(), new SwerveModuleState(),
      new SwerveModuleState(), new SwerveModuleState()
  };

  /** Creates a new DriveSubsystem. */
  public DriveSubsystem() {
    HAL.report(tResourceType.kResourceType_RobotDrive, tInstances.kRobotDriveSwerve_MaxSwerve);
    SmartDashboard.putData("Field", m_field);

    // Load PathPlanner RobotConfig safely
    RobotConfig config = null;
    try {
      config = RobotConfig.fromGUISettings();
    } catch (Exception e) {
      e.printStackTrace();
    }

    if (config == null) {
      DriverStation.reportError(
          "PathPlanner settings.json missing! Open PathPlanner GUI → Robot Settings → Save to src/main/deploy/pathplanner/",
          false);
    } else {
      AutoBuilder.configure(
          this::getPose,
          this::resetOdometry,
          this::getRobotRelativeSpeeds,          // now sim-aware
          (speeds, ffs) -> driveRobotRelative(speeds),
          new PPHolonomicDriveController(
              new PIDConstants(5.0, 0.0, 0.0),
              new PIDConstants(5.0, 0.0, 0.0)
          ),
          config,
          () -> DriverStation.getAlliance().isPresent()
             && DriverStation.getAlliance().get() == DriverStation.Alliance.Red,
          this
      );
    }
  }

  @Override
  public void periodic() {
    m_odometry.update(
        Rotation2d.fromDegrees(m_gyro.getAngle(IMUAxis.kZ)),
        new SwerveModulePosition[] {
            m_frontLeft.getPosition(),
            m_frontRight.getPosition(),
            m_rearLeft.getPosition(),
            m_rearRight.getPosition()
        });

    m_field.setRobotPose(getPose());
  }

  @Override
  public void simulationPeriodic() {
    double now = Timer.getFPGATimestamp();
    double dt = now - m_lastTime;
    m_lastTime = now;

    // Use last commanded states (encoders don't change in sim)
    ChassisSpeeds speeds = DriveConstants.kDriveKinematics.toChassisSpeeds(m_lastSetStates);

    Pose2d pose = getPose();
    Rotation2d h = pose.getRotation();
    double cos = h.getCos(), sin = h.getSin();

    double vxField = speeds.vxMetersPerSecond * cos - speeds.vyMetersPerSecond * sin;
    double vyField = speeds.vxMetersPerSecond * sin + speeds.vyMetersPerSecond * cos;
    double omega   = speeds.omegaRadiansPerSecond;

    Pose2d newPose = new Pose2d(
        pose.getX() + vxField * dt,
        pose.getY() + vyField * dt,
        Rotation2d.fromRadians(h.getRadians() + omega * dt));

    m_odometry.resetPosition(
        newPose.getRotation(),
        new SwerveModulePosition[] {
            m_frontLeft.getPosition(),
            m_frontRight.getPosition(),
            m_rearLeft.getPosition(),
            m_rearRight.getPosition()
        },
        newPose
    );

    m_gyroSim.setGyroAngleZ(newPose.getRotation().getDegrees());
    m_field.setRobotPose(newPose);
  }

  /** === PathPlanner hooks === */
  private ChassisSpeeds getRobotRelativeSpeeds() {
    // In sim, derive speeds from last commanded states so the controller “sees” motion
    if (RobotBase.isSimulation()) {
      return DriveConstants.kDriveKinematics.toChassisSpeeds(m_lastSetStates);
    }
    return DriveConstants.kDriveKinematics.toChassisSpeeds(getModuleStates());
  }

  private void driveRobotRelative(ChassisSpeeds speeds) {
    // Route through setModuleStates so cache updates
    SwerveModuleState[] states = DriveConstants.kDriveKinematics.toSwerveModuleStates(speeds);
    setModuleStates(states);
  }

  private SwerveModuleState[] getModuleStates() {
    return new SwerveModuleState[] {
        m_frontLeft.getState(),
        m_frontRight.getState(),
        m_rearLeft.getState(),
        m_rearRight.getState()
    };
  }

  /** Pose getters/setters */
  public Pose2d getPose() { return m_odometry.getPoseMeters(); }

  public void resetOdometry(Pose2d pose) {
    m_odometry.resetPosition(
        Rotation2d.fromDegrees(m_gyro.getAngle(IMUAxis.kZ)),
        new SwerveModulePosition[] {
            m_frontLeft.getPosition(),
            m_frontRight.getPosition(),
            m_rearLeft.getPosition(),
            m_rearRight.getPosition()
        },
        pose);
  }

  /** Teleop drive (field-relative optional) */
  public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
    double xSpeedDelivered = xSpeed * DriveConstants.kMaxSpeedMetersPerSecond;
    double ySpeedDelivered = ySpeed * DriveConstants.kMaxSpeedMetersPerSecond;
    double rotDelivered    = rot    * DriveConstants.kMaxAngularSpeed;

    SwerveModuleState[] states = DriveConstants.kDriveKinematics.toSwerveModuleStates(
        fieldRelative
            ? ChassisSpeeds.fromFieldRelativeSpeeds(
                xSpeedDelivered, ySpeedDelivered, rotDelivered,
                Rotation2d.fromDegrees(m_gyro.getAngle(IMUAxis.kZ)))
            : new ChassisSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered));

    setModuleStates(states);
  }

  /** X-lock */
  public void setX() {
    SwerveModuleState[] states = new SwerveModuleState[] {
      new SwerveModuleState(0, Rotation2d.fromDegrees(45)),
      new SwerveModuleState(0, Rotation2d.fromDegrees(-45)),
      new SwerveModuleState(0, Rotation2d.fromDegrees(-45)),
      new SwerveModuleState(0, Rotation2d.fromDegrees(45))
    };
    setModuleStates(states);
  }

  /** Directly set module states */
  public void setModuleStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(
        desiredStates, DriveConstants.kMaxSpeedMetersPerSecond);

    m_frontLeft.setDesiredState(desiredStates[0]);
    m_frontRight.setDesiredState(desiredStates[1]);
    m_rearLeft.setDesiredState(desiredStates[2]);
    m_rearRight.setDesiredState(desiredStates[3]);

    // Cache for simulation integration
    m_lastSetStates = new SwerveModuleState[] {
        desiredStates[0], desiredStates[1], desiredStates[2], desiredStates[3]
    };
  }

  public void resetEncoders() {
    m_frontLeft.resetEncoders();
    m_rearLeft.resetEncoders();
    m_frontRight.resetEncoders();
    m_rearRight.resetEncoders();
  }

  public void zeroHeading() { m_gyro.reset(); }

  public double getHeading() {
    return Rotation2d.fromDegrees(m_gyro.getAngle(IMUAxis.kZ)).getDegrees();
  }

  public double getTurnRate() {
    return m_gyro.getRate(IMUAxis.kZ) * (DriveConstants.kGyroReversed ? -1.0 : 1.0);
  }
}
