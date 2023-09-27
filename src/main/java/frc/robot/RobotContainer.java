package frc.robot;

import frc.robot.commands.TeleopSwerve;
import frc.robot.commands.superstructure.SuperstructureCommands;
import frc.robot.commands.superstructure.OperatorPrefs.PickupMode;
import frc.robot.commands.superstructure.OperatorPrefs.ScoreLevel;
import frc.robot.commands.superstructure.StateManager;
import frc.robot.subsystems.super_structure.States;
import frc.robot.subsystems.super_structure.SuperStructure;
import frc.robot.subsystems.swerve.Swerve;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class RobotContainer {
    private final CommandXboxController driveController = new CommandXboxController(0);
    private final CommandXboxController operatorController = new CommandXboxController(1);

    private final int translationAxis = XboxController.Axis.kLeftY.value;
    private final int strafeAxis = XboxController.Axis.kLeftX.value;
    private final int rotationAxis = XboxController.Axis.kRightX.value;

    // private final JoystickButton zeroGyro = new JoystickButton(driveController,
    // XboxController.Button.kStart.value);

    private final Swerve swerve = new Swerve();
    private final SuperStructure superStructure = new SuperStructure();

    public RobotContainer() {
        configureDriverBindings();
        configureOperatorBindings();

        swerve.setDefaultCommand(
                new TeleopSwerve(
                        swerve,
                        () -> -driveController.getRawAxis(translationAxis),
                        () -> -driveController.getRawAxis(strafeAxis),
                        () -> -driveController.getRawAxis(rotationAxis)));
    }

    private void configureDriverBindings() {
        // Bumpers/Triggers
        driveController.rightBumper().onTrue(new SuperstructureCommands.TransitionToPlace(superStructure));
        driveController.leftBumper().onTrue(new SuperstructureCommands.TransitionToPickup(superStructure));
        driveController.leftTrigger().onTrue(new StateManager.CmdTransitionState(superStructure, States.STANDBY));
        driveController.rightTrigger().onTrue(new StateManager.CmdTransitionState(superStructure, States.HOME));


        driveController.pov(180).onTrue(new InstantCommand(() -> superStructure.runEndEffector(12.0), superStructure));
        driveController.pov(0).onTrue(new InstantCommand(() -> superStructure.runEndEffector(-12.0), superStructure));
        driveController.pov(90).onTrue(new InstantCommand(() -> superStructure.runEndEffector(0.0), superStructure));




        // Center Buttons
        driveController.start().onTrue(new InstantCommand(() -> swerve.zeroGyro()));
    }

    private void configureOperatorBindings() {
        //Face buttons
        operatorController.y().onTrue(new InstantCommand(
            () -> ScoreLevel.setCurrentLevel(ScoreLevel.HIGH)));
        operatorController.b().onTrue(new InstantCommand(
            () -> ScoreLevel.setCurrentLevel(ScoreLevel.MIDDLE)));
        driveController.a().onTrue(new InstantCommand(
            () -> ScoreLevel.setCurrentLevel(ScoreLevel.LOW)));
        driveController.x().onTrue(new StateManager.CmdTransitionState(superStructure, States.HOME));

        //Bumpers/Triggers
        operatorController.rightBumper().onTrue(new InstantCommand(
            () -> PickupMode.setCurrentMode(PickupMode.GROUND)));
        operatorController.leftBumper().onTrue(new InstantCommand(
            () -> PickupMode.setCurrentMode(PickupMode.STATION)));
        operatorController.leftTrigger().onTrue(new InstantCommand(
            () -> GamepieceMode.setCurrentMode(GamepieceMode.CUBE)));
        operatorController.rightTrigger().onTrue(new InstantCommand(
            () -> GamepieceMode.setCurrentMode(GamepieceMode.CONE)));
    }

    public Command getAutonomousCommand() {
        return new InstantCommand();
    }
}
