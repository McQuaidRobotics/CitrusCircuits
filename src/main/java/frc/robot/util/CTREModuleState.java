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
    double currAngle = scope0To360(currentAngle.getDegrees());
    double targetAngle = desiredState.angle.getDegrees() + 180;
    double targetSpeed = desiredState.speedMetersPerSecond;
    double targetAngleDelta = targetAngle - currAngle;
    if (Math.abs(targetAngleDelta) > 90){
        targetSpeed = -targetSpeed;
        targetAngle = targetAngleDelta > 90 ? targetAngle - 180 : targetAngle + 180;
    }
    return new SwerveModuleState(targetSpeed, Rotation2d.fromDegrees(targetAngle));
  } 

  public static double scope0To360(double angle) {
    //passing 360 returnss 0
    if (angle < 0) {
      angle = 360-(Math.abs(angle)%360);
    } 
    else {
      angle %= 360;
    }
    return angle;
  }
}
