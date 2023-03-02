package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Drive;

public class BLESSEDAuto extends SequentialCommandGroup {
    public BLESSEDAuto(Drive drive) {
        super(
            new PrintCommand("WWWWW")
        );
    }
}
