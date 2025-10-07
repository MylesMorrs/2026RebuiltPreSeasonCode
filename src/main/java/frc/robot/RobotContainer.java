package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

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
    // Register NamedCommands here if needed (before building chooser)
    // NamedCommands.registerCommand("Example", Commands.print("Hi"));

    // Build auto chooser with a DEFAULT auto name that matches a .auto file
    autoChooser = AutoBuilder.buildAutoChooser("L3 CoralScoring");
    SmartDashboard.putData("Auto Chooser", autoChooser);

    configureButtonBindings();

    drive.setDefaultCommand(new RunCommand(() -> {
      double db = OIConstants.kDriveDeadband;
      double x   = -MathUtil.applyDeadband(driver.getLeftY(),  db);
      double y   = -MathUtil.applyDeadband(driver.getLeftX(),  db);
      double rot = -MathUtil.applyDeadband(driver.getRightX(), db);
      drive.drive(x, y, rot, fieldRelative);
    }, drive));

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
    // Fallback to the default name in case chooser didn't populate
    return AutoBuilder.buildAuto("L3 CoralScoring");
  }
}
