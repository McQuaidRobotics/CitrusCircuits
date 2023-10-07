package frc.robot.commands.swerve;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.commands.Helpers;
import frc.robot.subsystems.swerve.Swerve;

public class commandDriveOverChargeStation extends CommandBase{
    private Swerve swerve;
    private boolean towardsCommunity;
    private boolean hasLeftGround;
    private double leftGroundErrorDegrees;
    private double returnedToGroundErrorDegrees;
    private double driveStrengthPercent;
    private Debouncer leftGroundDebouncer;
    private Debouncer returnedToGroundDebouncer;

    /**
     * @param swerve The swerve subsytem to be used
     * @param towardsCommunity Whether the robot is driving over the charge station
     * towards it community or driving over the charge station away from its community.
     * @param leftGroundErrorDegrees The error in degrees from 0 the robot has to tilt above and
     * stay above before being considered off the ground
     * @param returnedToGroundErrorDegrees The error in degrees from 0 the robot has to tilt below and
     * stay below before being considered back on the ground
     * @param driveStrengthPercent The percentage of max speed to drive between 0 and 1:
     * EX: (1 = Max speed, 0.5 = Half of max speed)
     */
    public commandDriveOverChargeStation(
        Swerve swerve, 
        boolean towardsCommunity, 
        double leftGroundErrorDegrees,
        double returnedToGroundErrorDegrees,
        double driveStrengthPercent) {
        this.swerve = swerve;
        this.towardsCommunity = towardsCommunity;
        this.leftGroundErrorDegrees = leftGroundErrorDegrees;
        this.returnedToGroundErrorDegrees = returnedToGroundErrorDegrees;
        this.driveStrengthPercent = Helpers.clamp(driveStrengthPercent, 0.0, 1.0);
        addRequirements(swerve);
    }

    @Override
    public void initialize() {
        leftGroundDebouncer = new Debouncer(0.1, DebounceType.kRising);
        returnedToGroundDebouncer = new Debouncer(0.1, DebounceType.kRising);
    }

    @Override
    public void execute() {
        if (towardsCommunity) {
            hasLeftGround = leftGroundDebouncer.calculate(swerve.getRoll() >= Math.abs(leftGroundErrorDegrees));
            new TeleopSwerve(
                swerve, 
            () -> 0.0, 
            () -> 1 * driveStrengthPercent, 
            () -> 0.0,
            false);

            SmartDashboard.putBoolean(
            "returnedToGroundDebouncer | OverChargeStation", 
            returnedToGroundDebouncer.calculate(
            hasLeftGround && 
            (swerve.getRoll() >= -1.0 * Math.abs(returnedToGroundErrorDegrees))));
        }
        else {
            hasLeftGround = leftGroundDebouncer.calculate(swerve.getRoll() <= -1.0 * Math.abs(leftGroundErrorDegrees));
            new TeleopSwerve(
                swerve, 
            () -> 0.0, 
            () -> -1 * driveStrengthPercent, 
            () -> 0.0,
            false);

            SmartDashboard.putBoolean(
            "returnedToGroundDebouncer | OverChargeStation", 
            returnedToGroundDebouncer.calculate(
            hasLeftGround && 
            (swerve.getRoll() <= Math.abs(returnedToGroundErrorDegrees))));
        }

        SmartDashboard.putBoolean("towardCommunity | OverChargeStation", towardsCommunity);
        SmartDashboard.putBoolean("hasLeftGround | OverChargeStation", hasLeftGround);
        SmartDashboard.putNumber("leftGroundErrorDegrees | OverChargeStation", leftGroundErrorDegrees);
        SmartDashboard.putNumber("returnedToGroundErrorDegrees | OverChargeStation", returnedToGroundErrorDegrees);
        SmartDashboard.putNumber("driveStrengthPercent | OverChargeStation", driveStrengthPercent);
    }

    @Override
    public void end(boolean interrupted) {
        swerve.commandStopDrives();
    }

    @Override
    public boolean isFinished() {
        if (towardsCommunity) return returnedToGroundDebouncer.calculate(
            hasLeftGround && 
            (swerve.getRoll() >= -1.0 * Math.abs(returnedToGroundErrorDegrees)));

        return returnedToGroundDebouncer.calculate(
            hasLeftGround && 
            (swerve.getRoll() <= Math.abs(returnedToGroundErrorDegrees)));
    }
}