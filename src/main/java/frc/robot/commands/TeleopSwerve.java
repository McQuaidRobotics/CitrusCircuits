package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.subsystems.swerve.Swerve;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;

/** An example command that uses an example subsystem. */
public class TeleopSwerve extends CommandBase {
  @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })

  private Swerve swerve;
  private DoubleSupplier translationAxisSup;
  private DoubleSupplier strafeAxisSup;
  private DoubleSupplier rotationAxisSup;

  public TeleopSwerve(Swerve swerve, DoubleSupplier translation, DoubleSupplier strafe, DoubleSupplier rotation) {
    this.swerve = swerve;
    addRequirements(swerve);

    this.translationAxisSup = translation;
    this.strafeAxisSup = strafe;
    this.rotationAxisSup = rotation;
  }

  @Override
  public void execute() {
    double translationVal = MathUtil.applyDeadband(translationAxisSup.getAsDouble(), 0.1);
    double strafeVal = MathUtil.applyDeadband(strafeAxisSup.getAsDouble(), 0.1);
    double rotationVal = MathUtil.applyDeadband(rotationAxisSup.getAsDouble(), 0.2);

    swerve.Drive(
        new Translation2d(translationVal, strafeVal).times(Constants.kSwerve.MAX_SPEED),
        rotationVal * Constants.kSwerve.MAX_ANGULAR_VELOCITY,
        true,
        false);
  }
}
