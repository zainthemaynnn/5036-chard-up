package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj2.command.Subsystem;

public class Intake implements Subsystem {
    private CANSparkMax motor;

    public Intake(CANSparkMax motor) {
        this.motor = motor;
        motor.enableVoltageCompensation(12.0);
        motor.setOpenLoopRampRate(1/2.0);
        motor.setClosedLoopRampRate(1/2.0);
    }

    public void driveVolts(double pow) {
        motor.setVoltage(pow);
    }

    public void drivePower(double pow, boolean ramped) {
        motor.setOpenLoopRampRate(ramped ? 1/2.0 : 1);
        motor.setClosedLoopRampRate(ramped ? 1/2.0 : 1);
        motor.set(pow);
    }
}
