package frc.robot.commands.swerve;

import frc.robot.Constants;
import frc.robot.subsystems.swerve.Swerve;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.*;

/** An example command that uses an example subsystem. */
public class TeleopSwerve extends CommandBase {
  @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })

  private Swerve swerve;
  private DoubleSupplier translationAxisSup;
  private DoubleSupplier strafeAxisSup;
  private DoubleSupplier rotationAxisSup;
  private boolean applyDeadband;

  public TeleopSwerve(Swerve swerve, DoubleSupplier translation, DoubleSupplier strafe, DoubleSupplier rotation) {
    this.swerve = swerve;
    addRequirements(swerve);

    this.translationAxisSup = translation;
    this.strafeAxisSup = strafe;
    this.rotationAxisSup = rotation;
    this.applyDeadband = true;
  }

  public TeleopSwerve(Swerve swerve, DoubleSupplier translation, DoubleSupplier strafe, DoubleSupplier rotation, boolean applyDeadband) {
    this.swerve = swerve;
    addRequirements(swerve);

    this.translationAxisSup = translation;
    this.strafeAxisSup = strafe;
    this.rotationAxisSup = rotation;
    this.applyDeadband = applyDeadband;
  }

  @Override
  public void execute() {
    double translationVal; 
    double strafeVal; 
    double rotationVal;

    if (applyDeadband) {
      translationVal = MathUtil.applyDeadband(translationAxisSup.getAsDouble(), ControllerConsts.LEFT_DEADBAND);
      strafeVal = MathUtil.applyDeadband(strafeAxisSup.getAsDouble(), ControllerConsts.LEFT_DEADBAND);
      rotationVal = MathUtil.applyDeadband(rotationAxisSup.getAsDouble(), ControllerConsts.RIGHT_DEADBAND);
    }
    else {
      translationVal = translationAxisSup.getAsDouble();
      strafeVal = strafeAxisSup.getAsDouble();
      rotationVal = rotationAxisSup.getAsDouble();
    }

    swerve.Drive(
        new Translation2d(translationVal, strafeVal).times(Constants.kSwerve.MAX_SPEED),
        rotationVal * Constants.kSwerve.MAX_ANGULAR_VELOCITY,
        true,
        false);
  }
}
