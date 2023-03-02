package frc.hid;

import java.util.function.DoubleSupplier;
import java.util.function.DoubleUnaryOperator;

import edu.wpi.first.wpilibj2.command.button.Trigger;

public class JoystickAxis extends Trigger {
    private DoubleSupplier valueGetter;
    private double deadband;
    private DoubleUnaryOperator transform = (x) -> Math.pow(x, 2.) * Math.signum(x);

    public JoystickAxis(DoubleSupplier valueGetter, double deadband, boolean inverted) {
        this.valueGetter = inverted ? () -> -valueGetter.getAsDouble() : valueGetter;
        this.deadband = deadband;
    }

    public JoystickAxis(DoubleSupplier valueGetter, double deadband) {
        this(valueGetter, deadband, false);
    }

    public boolean get() {
        return this.value() != 0;
    }

    private double deadbandFilter(double v) {
        return Math.abs(v) > deadband ? v : 0;
    }

    public double value() {
        return transform.applyAsDouble(deadbandFilter(valueGetter.getAsDouble()));
    }
}
