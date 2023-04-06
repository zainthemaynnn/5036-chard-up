package frc.robot.commands;

import java.util.Set;

import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.subsystems.Blinkin;
import frc.robot.subsystems.Blinkin.BlinkinColor;
import frc.robot.subsystems.Drive;

public class Taxi implements Command {
    private static final double POW_STATIC = .0;
    private static final double POW_SIN_SCALE = 0.4;
    private double NAT_DEG;
    private static final double A_THRESH = Math.toRadians(3.0);
    private static final int LEVEL_COUNTS_THRESHOLD = 5;
    private static final int BALANCE_COUNTS_THRESHOLD = 10;
    private boolean level = false;
    private boolean taxi = false;
    private boolean decel = false;

    private Drive drive;
    private Blinkin leds;
    private boolean balanced = false;
    private double targetDist = Units.feetToMeters(-12.0);
    private double driveSpeed;
    private double levelCounts = 0;
    private double balanceCounts = 0;
    private double decelCounts = 0;

    public Taxi(Drive drive, Blinkin leds, double driveSpeed) {
        this.drive = drive;
        this.leds = leds;
        this.driveSpeed = driveSpeed;
    }

    @Override
    public void initialize() {
        drive.setIdleMode(IdleMode.kCoast);
        drive.rampEnabled = true;
        NAT_DEG = drive.pitch();
    }

    @Override
    public void execute() {
        if (!taxi) {
            System.out.println(drive.centerDistMeters() +  ", " + targetDist);
            if (drive.centerDistMeters() < targetDist) {
                System.out.println("taxi");
                taxi = true;
                drive.arcadeDrive(0., 0.);
            } else {
                drive.arcadeDrive(-driveSpeed, 0.);
                return;
            }
        }

        if (!decel) {
            if (++decelCounts >= 10) {
                System.out.println("decel");
                decel = true;
                drive.setIdleMode(IdleMode.kBrake);
                drive.rampEnabled = false;
                drive.arcadeDrive(0., 0.);
            } else {
                return;
            }
        }
    }

    @Override
    public void end(boolean interrupted) {
        if (leds.containsColor(BlinkinColor.BALANCED)) {
            leds.removeColor(BlinkinColor.BALANCED);
        }
        leds.display();
    }

    @Override
    public Set<Subsystem> getRequirements() {
        return Set.of(drive);
    }
}
