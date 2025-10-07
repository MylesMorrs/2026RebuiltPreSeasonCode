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
  private final DriveSubsystem drive = new DriveSubsystem();
  private final XboxController driver = new XboxController(OIConstants.kDriverControllerPort);

  private final SendableChooser<Command> autoChooser;
  private boolean fieldRelative = true;

  public RobotContainer() {
    // Register NamedCommands here (optional) BEFORE building chooser
    // NamedCommands.registerCommand("Example", Commands.print("Hello"));

    autoChooser = AutoBuilder.buildAutoChooser(); // uses .auto files from src/main/deploy/pathplanner/
    SmartDashboard.putData("Auto Chooser", autoChooser);

    configureButtonBindings();

    drive.setDefaultCommand(new RunCommand(() -> {
      double db = OIConstants.kDriveDeadband;
      double x   = -MathUtil.applyDeadband(driver.getLeftY(),  db);
      double y   = -MathUtil.applyDeadband(driver.getLeftX(),  db);
      double rot = -MathUtil.applyDeadband(driver.getRightX(), db);
      drive.drive(x, y, rot, fieldRelative);
    }, drive));

    // (Optional) also show Field2d from the drive in SmartDashboard (already put in subsystem)
    SmartDashboard.putData("Field", drive.getField());
  }

  private void configureButtonBindings() {
    new JoystickButton(driver, XboxController.Button.kRightBumper.value)
        .whileTrue(new RunCommand(drive::setX, drive));

    new JoystickButton(driver, XboxController.Button.kY.value)
        .onTrue(new RunCommand(() -> fieldRelative = !fieldRelative).ignoringDisable(true));

    new JoystickButton(driver, XboxController.Button.kStart.value)
        .onTrue(new RunCommand(drive::zeroHeading, drive));
  }

  public Command getAutonomousCommand() {
    Command chosen = autoChooser.getSelected();
    if (chosen != null) return chosen;

    // Fallback to a specific .auto by name if chooser is empty
    return AutoBuilder.buildAuto("L3 CoralScoring");
  }
}
