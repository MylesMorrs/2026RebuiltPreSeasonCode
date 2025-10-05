// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.DriveSubsystem;

public class RobotContainer {
  // Subsystems
  private final DriveSubsystem drive = new DriveSubsystem();

  // Controller
  private final XboxController driver = new XboxController(OIConstants.kDriverControllerPort);

  // PathPlanner chooser
  private final SendableChooser<Command> autoChooser;

  // Field-relative toggle
  private boolean fieldRelative = true;

  public RobotContainer() {
    // ---- Register NamedCommands here (optional) BEFORE building the chooser ----
    // Example (uncomment once you have commands):
    // NamedCommands.registerCommand("Intake", intakeSubsystem.intakeCommand());
    // NamedCommands.registerCommand("Shoot", shooterSubsystem.shootCommand());

    // Build & publish the auto chooser (shows .auto files from src/main/deploy/pathplanner/)
    autoChooser = AutoBuilder.buildAutoChooser();               // or buildAutoChooser("My Default Auto")
    SmartDashboard.putData("Auto Chooser", autoChooser);

    // Default teleop drive
    drive.setDefaultCommand(new RunCommand(() -> {
      double deadband = OIConstants.kDriveDeadband;
      double x   = -MathUtil.applyDeadband(driver.getLeftY(),  deadband);
      double y   = -MathUtil.applyDeadband(driver.getLeftX(),  deadband);
      double rot = -MathUtil.applyDeadband(driver.getRightX(), deadband);
      drive.drive(x, y, rot, fieldRelative);
    }, drive));

    // Buttons
    configureButtonBindings();
  }

  private void configureButtonBindings() {
    // Hold Right Bumper to X-lock
    new JoystickButton(driver, XboxController.Button.kRightBumper.value)
        .whileTrue(new RunCommand(drive::setX, drive));

    // Press Y to toggle field-relative on/off
    new JoystickButton(driver, XboxController.Button.kY.value)
        .onTrue(new RunCommand(() -> fieldRelative = !fieldRelative).ignoringDisable(true));

    // Press Start to zero heading
    new JoystickButton(driver, XboxController.Button.kStart.value)
        .onTrue(new RunCommand(drive::zeroHeading, drive));
  }

  /** Return the autonomous command. */
  public Command getAutonomousCommand() {
    Command chosen = autoChooser.getSelected();
    if (chosen != null) return chosen;

    // Fallback to a specific PathPlanner auto by name (must match your .auto file)
    return AutoBuilder.buildAuto("L3 CoralScoring");
  }
}
