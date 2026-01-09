// src/main/java/frc/robot/subsystems/ExampleSubsystem.java
package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;

import frc.robot.Constants.ExampleConstants;

public class ExampleSubsystem extends SubsystemBase {
  private final CANSparkMax motor = new CANSparkMax(ExampleConstants.kExampleMotorCanId, MotorType.kBrushless);
  private final RelativeEncoder encoder = motor.getEncoder();
  private final SparkMaxPIDController pid = motor.getPIDController();

  private double targetRotations = ExampleConstants.kTargetRotations;

  public ExampleSubsystem() {
    motor.restoreFactoryDefaults();
    motor.setIdleMode(IdleMode.kBrake);
    motor.setSmartCurrentLimit(40);
    motor.setInverted(false);

    // encoder units in OUTPUT rotations (set your gearing factor here)
    encoder.setPositionConversionFactor(ExampleConstants.kPositionConversionFactor);
    encoder.setPosition(0.0);

    // PID gains (tune on robot)
    pid.setP(ExampleConstants.kP);
    pid.setI(ExampleConstants.kI);
    pid.setD(ExampleConstants.kD);

    motor.burnFlash();
  }

  /** Set setpoint in OUTPUT rotations. */
  public void setTargetRotations(double rotations) {
    rotations = Math.max(ExampleConstants.kMinRotations, Math.min(ExampleConstants.kMaxRotations, rotations));
    targetRotations = rotations;
  }

  /** Set setpoint in OUTPUT degrees. */
  public void setTargetDegrees(double degrees) {
    setTargetRotations(degrees / 360.0);
  }

  public void zeroPosition() { encoder.setPosition(0.0); }
  public double getPositionRotations() { return encoder.getPosition(); }
  public double getTargetRotations() { return targetRotations; }

  @Override
  public void periodic() {
    pid.setReference(targetRotations, ControlType.kPosition);
    SmartDashboard.putNumber("Example/TargetRot", targetRotations);
    SmartDashboard.putNumber("Example/PositionRot", encoder.getPosition());
    SmartDashboard.putNumber("Example/ErrorRot", targetRotations - encoder.getPosition());
  }
}
