package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.subsystems.swerve.Swerve;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;

/** An example command that uses an example subsystem. */
public class TeleopSwerve extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})

  private Swerve swerve;
  private DoubleSupplier translationAxisSup;
  private DoubleSupplier strafeAxisSup;
  private DoubleSupplier rotationAxisSup;
  private BooleanSupplier robotCentricSup;

  public TeleopSwerve(Swerve swerve, DoubleSupplier translation, DoubleSupplier strafe, DoubleSupplier rotation, BooleanSupplier robotCentric) {
    this.swerve = swerve;
    addRequirements(swerve);

    this.translationAxisSup = translation;
    this.strafeAxisSup = strafe;
    this.rotationAxisSup = rotation;
    this.robotCentricSup = robotCentric;
  }

  @Override
  public void execute() {
    double translationVal = MathUtil.applyDeadband(translationAxisSup.getAsDouble(), Constants.OperatorConstants.leftStickDeadband);
    double strafeVal = MathUtil.applyDeadband(strafeAxisSup.getAsDouble(), Constants.OperatorConstants.leftStickDeadband);
    double rotationVal = MathUtil.applyDeadband(rotationAxisSup.getAsDouble(), Constants.OperatorConstants.rightStickDeadband);

    swerve.Drive(
        new Translation2d(translationVal, strafeVal).times(Constants.Swerve.maxSpeed), 
        rotationVal * Constants.Swerve.maxAngularVelocity, 
        robotCentricSup.getAsBoolean(), 
        false);
  }
}
