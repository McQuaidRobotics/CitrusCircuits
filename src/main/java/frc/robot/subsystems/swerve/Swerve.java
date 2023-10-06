package frc.robot.subsystems.swerve;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.kAuto;
import frc.robot.Constants.kSwerve;

public class Swerve extends SubsystemBase {
    private SwerveDriveOdometry swerveOdometry;
    private SwerveModule[] mSwerveMods;
    private Pigeon2 gyro;

    public Swerve() {
        this.gyro = new Pigeon2(Constants.kSwerve.PIGEON_ID, Constants.kSwerve.CANBUS);
        var gyroEmptyConfig = new Pigeon2Configuration();
        gyro.getConfigurator().apply(gyroEmptyConfig);
        zeroGyro();

        mSwerveMods = new SwerveModule[] {
                new SwerveModule(Constants.kSwerve.Mod0.CONSTANTS, 0),
                new SwerveModule(Constants.kSwerve.Mod1.CONSTANTS, 1),
                new SwerveModule(Constants.kSwerve.Mod2.CONSTANTS, 2),
                new SwerveModule(Constants.kSwerve.Mod3.CONSTANTS, 3)
        };

        Timer.delay(1.0);

        swerveOdometry = new SwerveDriveOdometry(Constants.kSwerve.SWERVE_KINEMATICS, getYaw(), getModulePositions());
    }

    public void Drive(Translation2d translation, double rotation, boolean fieldRelative) {
        SwerveModuleState[] mSwerveModuleStates = Constants.kSwerve.SWERVE_KINEMATICS.toSwerveModuleStates(fieldRelative
                ? ChassisSpeeds.fromFieldRelativeSpeeds(
                        translation.getX(),
                        translation.getY(),
                        rotation,
                        getYaw())
                : new ChassisSpeeds(
                        translation.getX(),
                        translation.getY(),
                        rotation));

        SwerveDriveKinematics.desaturateWheelSpeeds(mSwerveModuleStates, Constants.kSwerve.MAX_SPEED);

        for (SwerveModule module : mSwerveMods) {
            module.setDesiredState(mSwerveModuleStates[module.moduleNumber]);
        }
    }

    public Command commandPerpendicularDrives() {
        return new InstantCommand(() -> {
            SwerveModuleState[] newModuleStates = {
                new SwerveModuleState(0.0, Rotation2d.fromDegrees(0)),
                new SwerveModuleState(0.0, Rotation2d.fromDegrees(0)),
                new SwerveModuleState(0.0, Rotation2d.fromDegrees(0)),
                new SwerveModuleState(0.0, Rotation2d.fromDegrees(0))
            };

            for (SwerveModule module : mSwerveMods) {
                module.setAngle(null);
            }
        }).withName("commandPerpendicularDrives");
    }

    public Command commandStopDrives() {
        return new InstantCommand(() -> setModuleStates(new ChassisSpeeds())).withName("commandStopDrives");
    }

    public void zeroGyro() {
        gyro.setYaw(0.0);
    }

    public StatusSignal<Double> getGyroPitch() {
        return gyro.getPitch();
    }

    public Rotation2d getYaw() {
        return (Constants.kSwerve.INVERT_GYRO) ? Rotation2d.fromDegrees(360 - gyro.getYaw().getValue())
                : Rotation2d.fromDegrees(gyro.getYaw().getValue());
    }

    public SwerveModulePosition[] getModulePositions() {
        SwerveModulePosition[] modulePositions = new SwerveModulePosition[4];
        for (SwerveModule module : mSwerveMods) {
            modulePositions[module.moduleNumber] = module.getPosition();
        }
        return modulePositions;
    }

    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, Constants.kSwerve.MAX_SPEED);

        for (SwerveModule module : mSwerveMods) {
            module.setDesiredState(desiredStates[module.moduleNumber]);
        }
    }

    public void setModuleStates(ChassisSpeeds chassisSpeeds) {
        setModuleStates(kSwerve.SWERVE_KINEMATICS.toSwerveModuleStates(chassisSpeeds));
    }

    public SwerveModuleState[] getModuleStates() {
        SwerveModuleState[] states = new SwerveModuleState[4];
        for (SwerveModule module : mSwerveMods) {
            states[module.moduleNumber] = module.getState();
        }
        return states;
    }

    public Pose2d getPose() {
        return swerveOdometry.getPoseMeters();
    }

    public void resetOdometry(Pose2d pose) {
        swerveOdometry.resetPosition(getYaw(), getModulePositions(), pose);
    }

    public PathPlannerTrajectory openFilePath(String autoPathFile) {
        PathPlannerTrajectory autoPath = PathPlanner.loadPath(
            autoPathFile, 
            new PathConstraints(kSwerve.MAX_SPEED, kSwerve.MAX_ANGULAR_VELOCITY));
        return autoPath;
    }

    public Command commandRunPath(String autoPathFileName, boolean resetOdometry) {
        PathPlannerTrajectory autoPath = openFilePath(autoPathFileName);
        return new SequentialCommandGroup(
            new InstantCommand(() -> {
                if (resetOdometry) this.resetOdometry(autoPath.getInitialHolonomicPose());
            }),
            new PPSwerveControllerCommand(
                autoPath,
                this::getPose,
                kSwerve.SWERVE_KINEMATICS,
                kAuto.AUTO_X_PID,
                kAuto.AUTO_Y_PID,
                kAuto.AUTO_ANGULAR_PID,
                this::setModuleStates,
                this
            ),
            commandStopDrives()
        ).withName("commandRunPath: " + autoPathFileName);
    }

    @Override
    public void periodic() {
        for (SwerveModule module : mSwerveMods) {
            SmartDashboard.putNumber("Module " + module.moduleNumber + " Cancoder", module.getCanCoder().getDegrees());
            SmartDashboard.putNumber("Module " + module.moduleNumber + " Angle",
                    module.getPosition().angle.getDegrees());
            SmartDashboard.putNumber("Module " + module.moduleNumber + " Velocity",
                    module.getState().speedMetersPerSecond);
        }
    }
}
