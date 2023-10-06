package frc.robot.commands.swerve;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.commands.Helpers;
import frc.robot.subsystems.swerve.Swerve;

public class commandDriveTimedDirection extends CommandBase{
    private Swerve swerve;
    private Direction direction;
    private double duration;
    private double speedDampenPercent;
    private boolean resetGyro;
    private double timeOfInitilization;

    public enum Direction {
        LEFT(-1.0, 0.0),
        RIGHT(1.0, 0.0),
        BACKWARD(0.0, -1.0),
        FORWARD(0.0, 1.0);

        public double x, y;
        private Direction(double x, double y) {
            this.x = x;
            this.y = y;
        }
    }

    /**
     * @param swerve The swerve subsystem to be used
     * @param direction The direction x and y for the swerve to drive in
     * @param speedDampenPercent The percentage of max speed to drive between 0 and 1:
     * EX: (1 = Max speed, 0.5 = Half of max speed)
     * @param timeSeconds Time in seconds that the swerve will drive for
     * @param resetGyro Whether to reset the gyro when the command is scheduled or not
     */
    public commandDriveTimedDirection(Swerve swerve, Direction direction, Double speedDampenPercent, Double timeSeconds, boolean resetGyro) {
        this.swerve = swerve;
        this.direction = direction;
        this.duration = timeSeconds;
        this.resetGyro = resetGyro;
        this.speedDampenPercent = Helpers.clamp(speedDampenPercent, 0.0, 1.0);
        addRequirements(swerve);
    }

    @Override
    public void initialize() {
        timeOfInitilization = Timer.getFPGATimestamp();
        if (resetGyro) swerve.zeroGyro();
    }

    @Override
    public void execute() {
        new TeleopSwerve(
            swerve, 
            () -> (direction.x * speedDampenPercent), 
            () -> (direction.y * speedDampenPercent), 
            () -> 0.0,
            false);
    }

    @Override
    public boolean isFinished() {
        return (Timer.getFPGATimestamp() - timeOfInitilization) >= duration;
    }
}