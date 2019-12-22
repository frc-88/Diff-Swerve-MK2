/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import java.util.HashMap;
import java.util.Map;

import com.revrobotics.ControlType;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.team88.swerve.swervemodule.SwerveModule;
import frc.team88.swerve.swervemodule.motorsensor.PIDNeo;
import frc.team88.swerve.swervemodule.motorsensor.PIDTransmission;
import frc.team88.swerve.swervemodule.motorsensor.differential.DifferentialMechanism;
import frc.team88.swerve.util.WrappedAngle;
import frc.team88.swerve.util.constants.Constants;
import frc.team88.swerve.util.constants.PIDPreferenceConstants;

public class Robot extends TimedRobot {

  private HashMap<String, PIDNeo> neos;
  private HashMap<String, SwerveModule> modules;

  private PIDPreferenceConstants motorSpeedPIDConstants;
  private PIDPreferenceConstants azimuthPositionPIDConstants;

  private final static double azimuthGearRatio = 27.296 / 360.;
  private final static double wheelGearRatio = 8.31 / ((1. / 3.) * Math.PI);

  private NetworkTableEntry enableMotorVelocityControl;
  private NetworkTableEntry enableWheelSpeedControl;
  private NetworkTableEntry enableAzimuthVelocityControl;
  private NetworkTableEntry enableAzimuthPositionControl;

  private HashMap<String, NetworkTableEntry> entriesSensorMotorVelocity;
  private HashMap<String, NetworkTableEntry> entriesSetMotorVelocity;

  private HashMap<String, NetworkTableEntry> entriesSensorWheelSpeed;
  private HashMap<String, NetworkTableEntry> entriesSensorAzimuthVelocity;
  private HashMap<String, NetworkTableEntry> entriesSensorAzimuthPosition;
  private HashMap<String, NetworkTableEntry> entriesSetWheelSpeed;
  private HashMap<String, NetworkTableEntry> entriesSetAzimuthVelocity;
  private HashMap<String, NetworkTableEntry> entriesSetAzimuthPosition;

  private Joystick gamepad;
  private double angleTarget = 0;

  @Override
  public void robotInit() {
    // Initialize the PID constants
    motorSpeedPIDConstants = new PIDPreferenceConstants("Motor Speed", 0, 0.00000035, 0, 0.00019, 150, 0, 0);
    azimuthPositionPIDConstants = new PIDPreferenceConstants("Azimuth Position", 8.5, 0, 0.15, 0, 0, 0, 0);

    // Create the base NEOs
    neos = new HashMap<>();
    neos.put("fl+", new PIDNeo(16, motorSpeedPIDConstants));
    neos.put("fl-", new PIDNeo(1, motorSpeedPIDConstants));
    neos.put("bl+", new PIDNeo(2, motorSpeedPIDConstants));
    neos.put("bl-", new PIDNeo(3, motorSpeedPIDConstants));
    neos.put("br+", new PIDNeo(12, motorSpeedPIDConstants));
    neos.put("br-", new PIDNeo(13, motorSpeedPIDConstants));
    neos.put("fr+", new PIDNeo(14, motorSpeedPIDConstants));
    neos.put("fr-", new PIDNeo(15, motorSpeedPIDConstants));

    // Reversing all neos makes for counterclockise azimuth to be positve
    for (Map.Entry<String, PIDNeo> entry : neos.entrySet()) {
      entry.getValue().setInverted(true);
    }

    // Create the differentials
    DifferentialMechanism flDifferential = new DifferentialMechanism(neos.get("fl+"), neos.get("fl-"));
    DifferentialMechanism blDifferential = new DifferentialMechanism(neos.get("bl+"), neos.get("bl-"));
    DifferentialMechanism brDifferential = new DifferentialMechanism(neos.get("br+"), neos.get("br-"));
    DifferentialMechanism frDifferential = new DifferentialMechanism(neos.get("fr+"), neos.get("fr-"));

    // Create the transmissions
    PIDTransmission flAzimuthTransmission = new PIDTransmission(flDifferential.getSumMotor(), azimuthGearRatio);
    PIDTransmission flWheelTransmission = new PIDTransmission(flDifferential.getDifferenceMotor(), wheelGearRatio);
    PIDTransmission blAzimuthTransmission = new PIDTransmission(blDifferential.getSumMotor(), azimuthGearRatio);
    PIDTransmission blWheelTransmission = new PIDTransmission(blDifferential.getDifferenceMotor(), wheelGearRatio);
    PIDTransmission brAzimuthTransmission = new PIDTransmission(brDifferential.getSumMotor(), azimuthGearRatio);
    PIDTransmission brWheelTransmission = new PIDTransmission(brDifferential.getDifferenceMotor(), wheelGearRatio);
    PIDTransmission frAzimuthTransmission = new PIDTransmission(frDifferential.getSumMotor(), azimuthGearRatio);
    PIDTransmission frWheelTransmission = new PIDTransmission(frDifferential.getDifferenceMotor(), wheelGearRatio);

    // Set current positions to all be 0
    flAzimuthTransmission.calibratePosition(0);
    flWheelTransmission.calibratePosition(0);
    blAzimuthTransmission.calibratePosition(0);
    blWheelTransmission.calibratePosition(0);
    brAzimuthTransmission.calibratePosition(0);
    brWheelTransmission.calibratePosition(0);
    frAzimuthTransmission.calibratePosition(0);
    frWheelTransmission.calibratePosition(0);

    // Create the modules
    modules = new HashMap<>();
    modules.put("FL", new SwerveModule(flWheelTransmission, flAzimuthTransmission, flAzimuthTransmission,
        azimuthPositionPIDConstants));
    modules.put("BL", new SwerveModule(blWheelTransmission, blAzimuthTransmission, blAzimuthTransmission,
        azimuthPositionPIDConstants));
    modules.put("BR", new SwerveModule(brWheelTransmission, brAzimuthTransmission, brAzimuthTransmission,
        azimuthPositionPIDConstants));
    modules.put("FR", new SwerveModule(frWheelTransmission, frAzimuthTransmission, frAzimuthTransmission,
        azimuthPositionPIDConstants));

    // Create the SmartDashboard entries for control modes
    enableMotorVelocityControl = SmartDashboard.getEntry("E M V");
    enableMotorVelocityControl.setBoolean(false);
    enableWheelSpeedControl = SmartDashboard.getEntry("E W V");
    enableWheelSpeedControl.setBoolean(false);
    enableAzimuthVelocityControl = SmartDashboard.getEntry("E A V");
    enableAzimuthVelocityControl.setBoolean(false);
    enableAzimuthPositionControl = SmartDashboard.getEntry("E A P");
    enableAzimuthPositionControl.setBoolean(false);

    // Create the SmartDashboard entries for Neos
    entriesSensorMotorVelocity = new HashMap<>();
    entriesSetMotorVelocity = new HashMap<>();
    for (Map.Entry<String, PIDNeo> entry : neos.entrySet()) {
      entriesSensorMotorVelocity.put(entry.getKey(), SmartDashboard.getEntry(entry.getKey() + " R Vel"));
      entriesSensorMotorVelocity.get(entry.getKey()).setDouble(0);
      entriesSetMotorVelocity.put(entry.getKey(), SmartDashboard.getEntry(entry.getKey() + " S Vel"));
      entriesSetMotorVelocity.get(entry.getKey()).setDouble(0);
    }

    // Create the SmartDashboard entries for modules
    entriesSensorWheelSpeed = new HashMap<>();
    entriesSensorAzimuthVelocity = new HashMap<>();
    entriesSensorAzimuthPosition = new HashMap<>();
    entriesSetWheelSpeed = new HashMap<>();
    entriesSetAzimuthVelocity = new HashMap<>();
    entriesSetAzimuthPosition = new HashMap<>();
    for (Map.Entry<String, SwerveModule> entry : modules.entrySet()) {
      entriesSensorWheelSpeed.put(entry.getKey(), SmartDashboard.getEntry(entry.getKey() + " R W Vel"));
      entriesSensorWheelSpeed.get(entry.getKey()).setDouble(0);
      entriesSensorAzimuthVelocity.put(entry.getKey(), SmartDashboard.getEntry(entry.getKey() + " R A Vel"));
      entriesSensorAzimuthVelocity.get(entry.getKey()).setDouble(0);
      entriesSensorAzimuthPosition.put(entry.getKey(), SmartDashboard.getEntry(entry.getKey() + " R A Pos"));
      entriesSensorAzimuthPosition.get(entry.getKey()).setDouble(0);
      entriesSetWheelSpeed.put(entry.getKey(), SmartDashboard.getEntry(entry.getKey() + " S W Vel"));
      entriesSetWheelSpeed.get(entry.getKey()).setDouble(0);
      entriesSetAzimuthVelocity.put(entry.getKey(), SmartDashboard.getEntry(entry.getKey() + " S A Vel"));
      entriesSetAzimuthVelocity.get(entry.getKey()).setDouble(0);
      entriesSetAzimuthPosition.put(entry.getKey(), SmartDashboard.getEntry(entry.getKey() + " S A Pos"));
      entriesSetAzimuthPosition.get(entry.getKey()).setDouble(0);
    }

    gamepad = new Joystick(0);
  }

  @Override
  public void robotPeriodic() {
    Constants.update();

    for (Map.Entry<String, NetworkTableEntry> entry : entriesSensorMotorVelocity.entrySet()) {
      entry.getValue().setDouble(neos.get(entry.getKey()).getVelocity());
    }
    for (Map.Entry<String, NetworkTableEntry> entry : entriesSensorWheelSpeed.entrySet()) {
      entry.getValue().setDouble(modules.get(entry.getKey()).getWheelSpeed());
    }
    for (Map.Entry<String, NetworkTableEntry> entry : entriesSensorAzimuthVelocity.entrySet()) {
      entry.getValue().setDouble(modules.get(entry.getKey()).getAzimuthVelocity());
    }
    for (Map.Entry<String, NetworkTableEntry> entry : entriesSensorAzimuthPosition.entrySet()) {
      entry.getValue().setDouble(modules.get(entry.getKey()).getAzimuthPosition().getAngle());
    }
  }

  @Override
  public void autonomousInit() {
  }

  @Override
  public void autonomousPeriodic() {
  }

  @Override
  public void teleopInit() {
  }

  @Override
  public void teleopPeriodic() {
    modules.get("FL").setWheelSpeed(gamepad.getRawAxis(2) * 6.);
    if (gamepad.getMagnitude() > 0.9) {
      angleTarget = -gamepad.getDirectionDegrees();
    }
    modules.get("FL").setAzimuthPosition(new WrappedAngle(angleTarget));
  }

  @Override
  public void testInit() {
  }

  @Override
  public void testPeriodic() {
    if (enableMotorVelocityControl.getBoolean(false)) {
      for (Map.Entry<String, NetworkTableEntry> entry : entriesSetMotorVelocity.entrySet()) {
        neos.get(entry.getKey()).setVelocity(entry.getValue().getDouble(0));
      }
    }
    if (enableWheelSpeedControl.getBoolean(false)) {
      for (Map.Entry<String, NetworkTableEntry> entry : entriesSetWheelSpeed.entrySet()) {
        modules.get(entry.getKey()).setWheelSpeed(entry.getValue().getDouble(0));
      }
    }
    if (enableAzimuthVelocityControl.getBoolean(false)) {
      for (Map.Entry<String, NetworkTableEntry> entry : entriesSetAzimuthVelocity.entrySet()) {
        modules.get(entry.getKey()).setAzimuthVelocity(entry.getValue().getDouble(0));
      }
    }
    if (enableAzimuthPositionControl.getBoolean(false)) {
      for (Map.Entry<String, NetworkTableEntry> entry : entriesSetAzimuthPosition.entrySet()) {
        modules.get(entry.getKey()).setAzimuthPosition(new WrappedAngle(entry.getValue().getDouble(0)));
      }
    }
  }
}
