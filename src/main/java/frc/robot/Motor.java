package frc.robot;

import java.util.function.DoubleSupplier;

import com.revrobotics.CANSparkMax;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Motor {

    private String name;
    private CANSparkMax controller;
    private DoubleSupplier currentGetter;

    private NetworkTableEntry entryPercentVbus;
    private NetworkTableEntry entryVel;
    private NetworkTableEntry entryCurrent;

    public Motor(String name, CANSparkMax controller, DoubleSupplier currentGetter) {
        this.name = name;
        this.controller = controller;
        this.currentGetter = currentGetter;

        entryPercentVbus = SmartDashboard.getEntry(name + " vbus");
        entryPercentVbus.setDouble(0);
        entryVel = SmartDashboard.getEntry(name + " vel");
        entryVel.setDouble(0);
        entryCurrent = SmartDashboard.getEntry(name + " current");
        entryCurrent.setDouble(0);
    }

    public void update() {
        this.controller.set(entryPercentVbus.getDouble(0));
        entryCurrent.setDouble(currentGetter.getAsDouble());
        entryVel.setDouble(controller.getEncoder().getVelocity());
    }
}