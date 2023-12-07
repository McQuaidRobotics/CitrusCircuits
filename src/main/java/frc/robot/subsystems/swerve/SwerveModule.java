package frc.robot.subsystems.swerve;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

public interface SwerveModule {
    /**
     * @param desiredState The state that the module should assume, angle and velocity.
     * @param isOpenLoop Whether the module speed assumed should be reached via open or closed loop control.
     */
    public void setDesiredState(SwerveModuleState desiredState, boolean isOpenLoop);

    public void setAngle(SwerveModuleState desiredState);
    public void setSpeed(SwerveModuleState desiredState, boolean isOpenLoop);

    /**
     * @return The position/angle of the module.
     */
    public SwerveModulePosition getPosition();

    /**
     * @return The velocity/angle of the module.
     */
    public SwerveModuleState getState();

    public Rotation2d getAngle();

    /**
     * @return Returns the module's assigned number in the {@link Swerve#swerveMods} array.
     */
    public int getModuleNumber();


    default public void periodic(){}
    default public void simulationPeriodic(){}
}