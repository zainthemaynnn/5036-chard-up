package frc.robot.commands;

import java.security.Security;
import java.util.Set;

import javax.xml.transform.Source;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.subsystems.Arm;

public class ArmOut implements Command {
    private Arm arm;
    private static final double OUT = 2.27;

    public ArmOut(Arm arm) {
        this.arm = arm;
    }

    @Override
    public void initialize() {
        arm.drivePower(.25);
    }

    @Override
    public void end(boolean interrupted) {
        if (!interrupted) {
            arm.drivePower(0.);
        }
    }

    @Override
    public boolean isFinished() {
        return arm.position() > OUT;
    }

    @Override
    public Set<Subsystem> getRequirements() {
        return Set.of(arm);
    }
}
