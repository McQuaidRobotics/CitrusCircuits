package frc.robot;

import frc.robot.commands.TeleopSwerve;
import frc.robot.commands.superstructure.Commands;
import frc.robot.subsystems.super_structure.SuperStructure;
import frc.robot.subsystems.swerve.Swerve;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

public class RobotContainer {
    private final XboxController driveController = new XboxController(
            Constants.OperatorConstants.DRIVER_CONTROLLER_PORT);

    private final int translationAxis = XboxController.Axis.kLeftY.value;
    private final int strafeAxis = XboxController.Axis.kLeftX.value;
    private final int rotationAxis = XboxController.Axis.kRightX.value;

    private final JoystickButton zeroGyro = new JoystickButton(driveController, XboxController.Button.kStart.value);

    private final Swerve swerve = new Swerve();
    private final SuperStructure superStructure = new SuperStructure();

    public RobotContainer() {
        // configureDriverBindings();

        // swerve.setDefaultCommand(
        // new TeleopSwerve(
        // swerve,
        // () -> -driveController.getRawAxis(translationAxis),
        // () -> -driveController.getRawAxis(strafeAxis),
        // () -> -driveController.getRawAxis(rotationAxis)
        // ));

        superStructure.setDefaultCommand(
            Commands.manualControl(
                superStructure,
                // () -> driveController.getRawAxis(XboxController.Axis.kRightY.value),
                () -> 0.0,
                () -> driveController.getRawAxis(XboxController.Axis.kLeftY.value),
                // () -> 0.0,
                // () -> driveController.getLeftTriggerAxis() - driveController.getRightTriggerAxis(),
                () -> driveController.getRawAxis(XboxController.Axis.kRightY.value),
                () -> {
                    if (driveController.getAButton() && driveController.getBButton()) {
                        return 0.0;
                    }
                    if (driveController.getAButton()) {
                        return 1.0;
                    }
                    if (driveController.getBButton()) {
                        return -1.0;
                    }
                    return 0.0;
                }
            )
        );
    }

    // private void configureDriverBindings() {
    // zeroGyro.onTrue(new InstantCommand(() -> swerve.zeroGyro()));
    // }

    public Command getAutonomousCommand() {
        return new InstantCommand();
    }
}
