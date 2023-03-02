package frc.robot.commands;

import java.util.Set;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.subsystems.Drive;

public class TurnThatWorksAuto implements Command {
    private Drive drive;
    private double initial;
    private double dist;
    private double thresh;

    public TurnThatWorksAuto(Drive drive, double dist, double thresh) {
        this.drive = drive;
        this.dist = dist;
        this.thresh = thresh;
    }

    @Override
    public void initialize() {
        initial = drive.centerDist();
        drive.arcadeDrive(0, .25 * Math.signum(dist));
    }

    @Override
    public boolean isFinished() {
        return Math.abs(drive.angle() - initial) < thresh;
    }

    @Override
    public Set<Subsystem> getRequirements() {
        return Set.of(drive);
    }
}
