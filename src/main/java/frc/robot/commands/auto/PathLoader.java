package frc.robot.commands.auto;


import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.SwerveAutoBuilder;

import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.Constants.kAuto;
import frc.robot.Constants.kSwerve;

public class PathLoader {
    public static PathPlannerTrajectory openFilePath(String autoPathFile) {
        PathPlannerTrajectory autoPath = PathPlanner.loadPath(
            autoPathFile, 
            new PathConstraints(kSwerve.MAX_SPEED, kSwerve.MAX_ANGULAR_VELOCITY));
        return autoPath;
    }

    public static SwerveAutoBuilder getPPAutoBuilder() {
        return new SwerveAutoBuilder(
            RobotContainer.swerve::getPose, 
            RobotContainer.swerve::resetOdometry, 
            Constants.kSwerve.SWERVE_KINEMATICS,
            new PIDConstants(
                kAuto.AUTO_TRANSLATION_PID.getP(), 
                kAuto.AUTO_TRANSLATION_PID.getI(), 
                kAuto.AUTO_TRANSLATION_PID.getD()), 
            new PIDConstants(
                kAuto.AUTO_ANGULAR_PID.getP(), 
                kAuto.AUTO_ANGULAR_PID.getI(), 
                kAuto.AUTO_ANGULAR_PID.getD()), 
            RobotContainer.swerve::setModuleStates, 
            Blocks.EVENT_MAP,
            true,
            RobotContainer.swerve);
    }
}
