package frc.robot.commands;

import java.util.Set;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Vision;

public class Align implements Command {
    private static final double kP = 0.5;
    private Vision vision;
    private Drive drive;
    private double latest;
    private double dir;
    private boolean detected = false;

    public Align(Drive drive, Vision vision, double dir) {
        this.drive = drive;
        this.vision = vision;
        this.dir = dir;
    }

    private boolean hasTarget() {
        return vision.hasTarget() && (vision.poseTagRelative().getZ() > 0.1);
    }

    private double angle() {
        var pose = vision.poseTagRelative();
        return pose.getRotation().getY();
    }

    private void lock() {
        detected = true;
        latest = drive.angle() + angle();
        System.out.printf("%f %f\n", drive.angle(), latest);
    }

    @Override
    public void initialize() {
        drive.rampEnabled = false;
        if (hasTarget()) {
            System.out.println("init");
            lock();
        } else {
            System.out.println("cancel");
        }
    }

    @Override
    public void execute() {
        if (detected) {
            drive.arcadeDrive(0., kP * (latest - drive.angle()));
        } else {
            drive.arcadeDrive(0., .2 * dir);
            if (hasTarget()) {
                lock();
            }
        }
    }

    @Override
    public void end(boolean interrupted) {
        drive.arcadeDrive(0., 0.);
        System.out.printf("%f %f\n", drive.angle(), latest);
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public Set<Subsystem> getRequirements() {
        return Set.of(drive, vision);
    }
}
