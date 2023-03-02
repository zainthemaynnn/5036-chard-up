package frc.robot.commands;

import java.util.Set;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.subsystems.Arm;

public class ArmIn implements Command {
    private Arm arm;
    private static final double IN = 0.01;

    public ArmIn(Arm arm) {
        this.arm = arm;
    }

    @Override
    public void initialize() {
        arm.drivePower(-.25);
    }

    @Override
    public void end(boolean interrupted) {
        if (!interrupted) {
            arm.drivePower(0.);
        }
    }

    @Override
    public boolean isFinished() {
        return arm.position() > IN;
    }

    @Override
    public Set<Subsystem> getRequirements() {
        return Set.of(arm);
    }
}
