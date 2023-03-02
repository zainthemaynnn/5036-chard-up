package frc.ui;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableRegistry;

public class Dashboard {
    String subsystem;

    public Dashboard(String subsystem) {
        this.subsystem = subsystem;
    }

    public Dashboard add(Sendable obj, String name) {
        SendableRegistry.addLW(obj, subsystem, name);
        return this;
    }
}
