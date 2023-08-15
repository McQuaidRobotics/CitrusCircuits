package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.subsystems.swerve.Swerve;

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

  public TeleopSwerve(Swerve swerve, DoubleSupplier translation, DoubleSupplier strafe, DoubleSupplier rotation) {
    this.swerve = swerve;
    addRequirements(swerve);

    this.translationAxisSup = translation;
    this.strafeAxisSup = strafe;
    this.rotationAxisSup = rotation;
  }

  @Override
  public void execute() {
    double translationVal = MathUtil.applyDeadband(translationAxisSup.getAsDouble(), Constants.OperatorConstants.LEFT_JOYSTICK_DAEDBAND);
    double strafeVal = MathUtil.applyDeadband(strafeAxisSup.getAsDouble(), Constants.OperatorConstants.LEFT_JOYSTICK_DAEDBAND);
    double rotationVal = MathUtil.applyDeadband(rotationAxisSup.getAsDouble(), Constants.OperatorConstants.RIGHT_JOYSTICK_DEADBAND);

    swerve.Drive(
        new Translation2d(translationVal, strafeVal).times(Constants.Swerve.MAX_SPEED), 
        rotationVal * Constants.Swerve.MAX_ANGULAR_VELOCITY, 
        true, 
        false);
  }
}
