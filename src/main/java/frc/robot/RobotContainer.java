package frc.robot;

import frc.robot.commands.TeleopSwerve;
import frc.robot.subsystems.swerve.Swerve;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

public class RobotContainer {
  private final XboxController driveController = new XboxController(Constants.OperatorConstants.DRIVER_CONTROLLER_PORT);

  private final int translationAxis = XboxController.Axis.kLeftY.value;
  private final int strafeAxis = XboxController.Axis.kLeftX.value;
  private final int rotationAxis = XboxController.Axis.kRightX.value;

  private final JoystickButton zeroGyro = new JoystickButton(driveController, XboxController.Button.kStart.value);

  private final Swerve swerve = new Swerve();

  public RobotContainer() {
    configureDriverBindings();

    swerve.setDefaultCommand(
      new TeleopSwerve(
        swerve,
        () -> -driveController.getRawAxis(translationAxis),
        () -> -driveController.getRawAxis(strafeAxis),
        () -> -driveController.getRawAxis(rotationAxis)
        ));
  }

  private void configureDriverBindings() {
    zeroGyro.onTrue(new InstantCommand(() -> swerve.zeroGyro()));
  }

  public Command getAutonomousCommand() {
    return new InstantCommand();
  }
}
