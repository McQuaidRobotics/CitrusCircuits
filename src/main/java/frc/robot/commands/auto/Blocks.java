package frc.robot.commands.auto;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;

public class Blocks {

    static CommandBase allianceConditionalCommand(CommandBase redCommand, CommandBase blueCommand) {
        return new ConditionalCommand(
            redCommand, blueCommand, () -> DriverStation.getAlliance() == DriverStation.Alliance.Red);
    }
}
