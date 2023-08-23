package frc.robot.util;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;

public class CTREModuleState {

  /**
   * Minimize the change in heading the desired swerve module state would require by potentially
   * reversing the direction the wheel spins. Customized from WPILib's version to include placing
   * in appropriate scope for CTRE onboard control.
   *
   * @param desiredState The desired state.
   * @param currentAngle The current module angle.
   */
  public static SwerveModuleState optimize(SwerveModuleState desiredState, Rotation2d currentAngle) {
    double targetAngle = desiredState.angle.getDegrees();
    double targetSpeed = desiredState.speedMetersPerSecond;
    double targetAngleDelta = targetAngle - currentAngle.getDegrees();
    if (targetAngleDelta > 90) {
      targetSpeed = -targetSpeed;
      targetAngle = targetAngle - 180;
      return new SwerveModuleState(targetSpeed, Rotation2d.fromDegrees(targetAngle));
    }
    else if (targetAngleDelta <-90) {
      targetSpeed = -targetSpeed;
      targetAngle = targetAngle + 180;
      return new SwerveModuleState(targetSpeed, Rotation2d.fromDegrees(targetAngle));
    }
    else {
      return new SwerveModuleState(targetSpeed, Rotation2d.fromDegrees(targetAngle));
    }
  }  
}
