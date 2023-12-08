package frc.robot.subsystems.swerve;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.controls.PositionDutyCycle;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.kSwerve;
import frc.robot.Constants.kSwerve.Sim;
import frc.robot.util.SwerveModuleConstants;

public class SwerveModuleSim implements SwerveModule{
    private FlywheelSim driveSim = new FlywheelSim(DCMotor.getFalcon500(1), 1.0 / kSwerve.DRIVE_MECHANISM_RATIO, 0.025);
    private FlywheelSim angleSim = new FlywheelSim(DCMotor.getFalcon500(1), 1.0 / kSwerve.ANGLE_MECHANISM_RATIO, 0.004);
    
    private final PIDController driveFeedback =
        new PIDController(0.0, 0.0, 0.0, 0.02);
    private final PIDController angleFeedback =
        new PIDController(0.0, 0.0, 0.0, 0.02);

    private double angleRelativePositionRad = 0.0;
    private double angleAbsolutePositionRad = Math.random() * 2.0 * Math.PI;
    private double driveAppliedVolts = 0.0;
    private double angleAppliedVolts = 0.0;
    private Rotation2d lastAngle = new Rotation2d();

    public int moduleNumber;

    public SwerveModuleSim(final SwerveModuleConstants moduleConstants) {
        this.moduleNumber = moduleConstants.moduleId.num;
        angleFeedback.enableContinuousInput(-Math.PI, Math.PI);
    }

    public void setDesiredState(SwerveModuleState desiredState, boolean isOpenLoop) {
        desiredState = SwerveModuleState.optimize(desiredState, getAngle());
        setAngle(desiredState);
        setSpeed(desiredState, isOpenLoop);
    }

    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition();
    }

    public SwerveModuleState getState() {
        return new SwerveModuleState();
    }

    public int getModuleNumber() {
        return this.moduleNumber;
    }

    public Rotation2d getAngle() {
        return lastAngle;
    }

    public void setAngle(SwerveModuleState desiredState) {
        Rotation2d angle = (Math.abs(desiredState.speedMetersPerSecond) <= (kSwerve.MAX_SPEED * 0.01)) ? lastAngle
                : desiredState.angle;

        SmartDashboard.putNumber("Module: " + moduleNumber + " angle", angle.getDegrees() + 180);

        angleAppliedVolts = MathUtil.clamp(
            angleFeedback.calculate(getAngle().getRadians(), angle.getRadians()), 
            -1.0 * RobotController.getBatteryVoltage(), 
            RobotController.getBatteryVoltage());
        angleSim.setInputVoltage(angleAppliedVolts);

        lastAngle = angle;
    }

    public void setSpeed(SwerveModuleState desiredState, boolean isOpenLoop) {
        if (isOpenLoop) {
            Logger.recordOutput("Module: " + moduleNumber + " velo", desiredState.speedMetersPerSecond);

            driveSim.setInputVoltage((desiredState.speedMetersPerSecond / kSwerve.MAX_SPEED) * Sim.MAX_VOLTAGE);
        }
        else {
            desiredState.speedMetersPerSecond *= Math.cos(angleFeedback.getPositionError());

            SmartDashboard.putNumber("Module: " + moduleNumber + " velo", desiredState.speedMetersPerSecond);

            double velocityRadPerSec = desiredState.speedMetersPerSecond / (kSwerve.WHEEL_DIAMETER / 2);
            driveAppliedVolts = MathUtil.clamp(
                driveFeedback.calculate(driveSim.getAngularVelocityRadPerSec(), velocityRadPerSec),
                -1.0 * RobotController.getBatteryVoltage(),
                RobotController.getBatteryVoltage());
            driveSim.setInputVoltage(driveAppliedVolts);
        }
    }

    @Override
    public void simulationPeriodic() {

    }
}
