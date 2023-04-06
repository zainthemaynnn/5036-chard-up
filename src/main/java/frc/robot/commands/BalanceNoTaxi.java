package frc.robot.commands;

import java.util.Set;

import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.subsystems.Blinkin;
import frc.robot.subsystems.Blinkin.BlinkinColor;
import frc.robot.subsystems.Drive;

public class BalanceNoTaxi implements Command {
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

    public BalanceNoTaxi(Drive drive, Blinkin leds, double driveSpeed) {
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
        double a = drive.pitch() - NAT_DEG;
        drive.arcadeDrive(-driveSpeed, 0.);
        if (!level) {
            if (Math.abs(a) > Math.toRadians(11.0)) {
                if (++levelCounts >= LEVEL_COUNTS_THRESHOLD) {
                    System.out.println("level");
                    level = true;
                }
            } else {
                levelCounts = 0;
                return;
            }
            
        }
        if (level) {
            if (Math.abs(a) < A_THRESH && ++balanceCounts >= BALANCE_COUNTS_THRESHOLD) {
                System.out.println("balance");
                balanced = true;
            } else {
                balanceCounts = 0;
            }
            boolean wasBalanced = balanced;
            var pow = !balanced ? POW_STATIC + POW_SIN_SCALE * -Math.sin(a) : 0.;
            drive.arcadeDrive(
                pow,
                0.
            );
            System.out.println(pow);
            if (balanced && !wasBalanced) {
                leds.addColor(BlinkinColor.BALANCED);
            } else if (!balanced && wasBalanced) {
                leds.removeColor(BlinkinColor.BALANCED);
            }
        } else {
            drive.arcadeDrive(-driveSpeed, 0.);
        }
        leds.display();
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
