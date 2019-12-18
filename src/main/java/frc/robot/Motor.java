package frc.robot;

import java.util.function.DoubleSupplier;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Motor {

    private String name;
    private SpeedController controller;
    private DoubleSupplier currentGetter;

    private NetworkTableEntry entryPercentVbus;
    private NetworkTableEntry entryCurrent;

    public Motor(String name, SpeedController controller, DoubleSupplier currentGetter) {
        this.name = name;
        this.controller = controller;
        this.currentGetter = currentGetter;

        entryPercentVbus = SmartDashboard.getEntry(name + " vbus");
        entryPercentVbus.setDouble(0);
        entryCurrent = SmartDashboard.getEntry(name + " current");
        entryCurrent.setDouble(0);
    }

    public void update() {
        this.controller.set(entryPercentVbus.getDouble(0));
        entryCurrent.setDouble(currentGetter.getAsDouble());
    }
}