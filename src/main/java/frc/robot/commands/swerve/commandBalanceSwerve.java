package frc.robot.commands.swerve;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.commands.Helpers;
import frc.robot.subsystems.swerve.Swerve;

public class commandBalanceSwerve extends CommandBase {
    private double pitchErrorDegrees;
    private Swerve swerve;
    private double forwardDriveStrengthPercent;
    private double backwardDriveStrengthPercent;
    private Debouncer balancedDebouncer;

    /**
     * @param swerve            The swerve subsystem to be used
     * @param pitchErrorDegrees The max error in degrees that is allowed
     *                          before being considered balanced
     * @param initalStrength    The percentage of max speed to drive between 0 and
     *                          1:
     *                          EX: (1 = Max speed, 0.5 = Half of max speed)
     */

    public commandBalanceSwerve(Swerve swerve, Double pitchErrorDegrees, Double initialStrengthPercent) {
        this.swerve = swerve;
        this.pitchErrorDegrees = pitchErrorDegrees;
        this.forwardDriveStrengthPercent = Helpers.clamp(initialStrengthPercent, 0.0, 1.0);
        this.backwardDriveStrengthPercent = Helpers.clamp(initialStrengthPercent, 0.0, 1.0);
        addRequirements(swerve);
    }

    @Override
    public void initialize() {
        this.balancedDebouncer = new Debouncer(0.1, DebounceType.kRising);
    }

    @Override
    public void execute() {
        // if (swerve.getPitch() > 0) {
        //     new TeleopSwerve(
        //             swerve,
        //             () -> -1.0 * backwardDriveStrengthPercent,
        //             () -> 0.0,
        //             () -> 0.0,
        //             false);
        // } else {
        //     new TeleopSwerve(
        //             swerve,
        //             () -> 1.0 * forwardDriveStrengthPercent,
        //             () -> 0.0,
        //             () -> 0.0,
        //             false);
        // }

        /*
         * TODO {Maddox} May want some kind of system in place which will decrease
         * the drive strength each time the robot switches direction while teetering
         * on the charge station
         */

        SmartDashboard.putNumber("pitchErrorDegrees | Balance", pitchErrorDegrees);
        SmartDashboard.putNumber("forwardDriveStrength | Balance", forwardDriveStrengthPercent);
        SmartDashboard.putNumber("backwardDriveStrength | Balance", backwardDriveStrengthPercent);
        SmartDashboard.putBoolean(
                "balanced | Balance",
                balancedDebouncer.calculate(Math.abs(swerve.getPitch()) <= pitchErrorDegrees));
    }

    @Override
    public void end(boolean interrupted) {
        swerve.commandXDrives();
    }

    @Override
    public boolean isFinished() {
        return balancedDebouncer.calculate(Math.abs(swerve.getPitch()) <= pitchErrorDegrees);
    }
}
