package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Subsystem;

public class Intake implements Subsystem {
    private CANSparkMax motor;

    public Intake(CANSparkMax motor) {
        this.motor = motor;
        motor.enableVoltageCompensation(12.0);
    }

    public void driveVolts(double pow) {
        motor.setVoltage(pow);
    }

    public void drivePower(double pow) {
        motor.set(pow);
    }
}
