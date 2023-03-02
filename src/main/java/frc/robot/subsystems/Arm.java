package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Subsystem;

public class Arm implements Subsystem {
    private static final double GEAR_RATIO = 80.;
    public static final double BACK = Math.toRadians(-72.);
    public static final double HIGH = Math.toRadians(0.);
    private CANSparkMax motor;
    private RelativeEncoder enc;

    public Arm(CANSparkMax motor) {
        this.motor = motor;
        this.enc = motor.getEncoder();
        //motor.setSmartCurrentLimit(60);
        motor.enableVoltageCompensation(12.0);
        enc.setPositionConversionFactor(2*Math.PI/GEAR_RATIO);
        enc.setPosition(0.);
    }

    public void driveVolts(double pow) {
        motor.setVoltage(pow);
    }

    public void drivePower(double pow) {
        motor.set(pow);
    }

    public double velocity() {
        return enc.getVelocity();
    }

    public double position() {
        return enc.getPosition();
    }
}
