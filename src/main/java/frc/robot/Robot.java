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
import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.I2C.Port;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.team88.swerve.SwerveChassis;
import frc.team88.swerve.swervemodule.SwerveModule;
import frc.team88.swerve.swervemodule.motorsensor.PIDNeo;
import frc.team88.swerve.swervemodule.motorsensor.PIDTransmission;
import frc.team88.swerve.swervemodule.motorsensor.differential.DifferentialMechanism;
import frc.team88.swerve.util.Vector2D;
import frc.team88.swerve.util.WrappedAngle;
import frc.team88.swerve.util.constants.Constants;
import frc.team88.swerve.util.constants.PIDPreferenceConstants;
import frc.team88.swerve.wrappers.gyro.NavX;
import jdk.jfr.Enabled;

public class Robot extends TimedRobot {

  private HashMap<String, PIDNeo> neos;
  private HashMap<String, PIDTransmission> transmissions;
  private HashMap<String, SwerveModule> modules;
  private NavX navx;
  private SwerveChassis chassis;

  private Joystick gamepad;

  private PIDPreferenceConstants motorSpeedPIDConstants;
  private PIDPreferenceConstants azimuthPositionPIDConstants;

  private static final double AZIMUTH_GEAR_RATIO = 27.296 / 360.;
  private static final double WHEEL_GEAR_RATIO = 8.31 / ((1. / 3.) * Math.PI);

  private static final double WIDTH = 24.75 / 12.;
  private static final double LENGTH = 21.75 / 12.;

  private static final double MAX_SPEED = 10;
  private static final double MAX_ROTATION = 360;

  private boolean calibrateMode = false;

  private WrappedAngle translationAngle = new WrappedAngle(0);

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
    transmissions = new HashMap<>();
    transmissions.put("FL Azimuth", new PIDTransmission(flDifferential.getSumMotor(), AZIMUTH_GEAR_RATIO));
    transmissions.put("FL Wheel", new PIDTransmission(flDifferential.getDifferenceMotor(), WHEEL_GEAR_RATIO));
    transmissions.put("BL Azimuth", new PIDTransmission(blDifferential.getSumMotor(), AZIMUTH_GEAR_RATIO));
    transmissions.put("BL Wheel", new PIDTransmission(blDifferential.getDifferenceMotor(), WHEEL_GEAR_RATIO));
    transmissions.put("BR Azimuth", new PIDTransmission(brDifferential.getSumMotor(), AZIMUTH_GEAR_RATIO));
    transmissions.put("BR Wheel", new PIDTransmission(brDifferential.getDifferenceMotor(), WHEEL_GEAR_RATIO));
    transmissions.put("FR Azimuth", new PIDTransmission(frDifferential.getSumMotor(), AZIMUTH_GEAR_RATIO));
    transmissions.put("FR Wheel", new PIDTransmission(frDifferential.getDifferenceMotor(), WHEEL_GEAR_RATIO));

    // Set current positions to all be 0
    transmissions.get("FL Azimuth").calibratePosition(0);
    transmissions.get("FL Wheel").calibratePosition(0);
    transmissions.get("BL Azimuth").calibratePosition(0);
    transmissions.get("BL Wheel").calibratePosition(0);
    transmissions.get("BR Azimuth").calibratePosition(0);
    transmissions.get("BR Wheel").calibratePosition(0);
    transmissions.get("FR Azimuth").calibratePosition(0);
    transmissions.get("FR Wheel").calibratePosition(0);

    // Create the modules
    modules = new HashMap<>();
    modules.put("FL", new SwerveModule(transmissions.get("FL Wheel"), transmissions.get("FL Azimuth"),
        transmissions.get("FL Azimuth"), azimuthPositionPIDConstants));
    modules.put("BL", new SwerveModule(transmissions.get("BL Wheel"), transmissions.get("BL Azimuth"),
        transmissions.get("BL Azimuth"), azimuthPositionPIDConstants));
    modules.put("BR", new SwerveModule(transmissions.get("BR Wheel"), transmissions.get("BR Azimuth"),
        transmissions.get("BR Azimuth"), azimuthPositionPIDConstants));
    modules.put("FR", new SwerveModule(transmissions.get("FR Wheel"), transmissions.get("FR Azimuth"),
        transmissions.get("FR Azimuth"), azimuthPositionPIDConstants));

    // Set the module locations
    modules.get("FL").setLocation(Vector2D.createCartesianCoordinates(-WIDTH / 2, LENGTH / 2));
    modules.get("BL").setLocation(Vector2D.createCartesianCoordinates(-WIDTH / 2, -LENGTH / 2));
    modules.get("BR").setLocation(Vector2D.createCartesianCoordinates(WIDTH / 2, -LENGTH / 2));
    modules.get("FR").setLocation(Vector2D.createCartesianCoordinates(WIDTH / 2, LENGTH / 2));

    // Create and zero gyro
    navx = new NavX(Port.kOnboard);
    navx.calibrateYaw(0);

    // Create the chassis
    chassis = new SwerveChassis(navx, modules.get("FL"), modules.get("BL"), modules.get("BR"), modules.get("FR"));
    chassis.setMaxWheelSpeed(MAX_SPEED);

    // Create the gamepad
    gamepad = new Joystick(0);
  }

  @Override
  public void robotPeriodic() {
    if (gamepad.getRawButton(7)) {
      // OOOOOOOH, I WONDER WHAT THIS DOES!
      int infiniteTJSquareds = 88 / 0;
      throw new IllegalStateException(
          infiniteTJSquareds + " is too much awesome, I don't know how we didn't crash already!");
    }

    Constants.update();
    SmartDashboard.putNumber("Yaw", navx.getYaw());
  }

  @Override
  public void disabledInit() {
  }

  @Override
  public void disabledPeriodic() {
    if (gamepad.getRawButton(1)) {
      enableCalibrateMode();
    }
    if (gamepad.getRawButton(4)) {
      disableCalibrateMode();
    }
  }

  @Override
  public void autonomousInit() {
    disableCalibrateMode();
  }

  @Override
  public void autonomousPeriodic() {
  }

  @Override
  public void teleopInit() {
    disableCalibrateMode();
  }

  @Override
  public void teleopPeriodic() {
    // Check robot-centric mode
    if (gamepad.getRawButton(1)) {
      chassis.setRobotCentic();
    }

    // Check field-centric mode
    if (gamepad.getRawButton(2)) {
      chassis.setFieldCentic();
    }

    // Check hammer mode
    if (gamepad.getRawButton(5)) {
      chassis.enableHammerMode();
    } else {
      chassis.disableHammerMode();
    }

    // Check if the translation angle should be updated
    if (gamepad.getMagnitude() > 0.9) {
      this.translationAngle = new WrappedAngle(-gamepad.getDirectionDegrees());
    }

    // Determine the translation speed
    double translationSpeed = gamepad.getRawAxis(3) * (gamepad.getRawAxis(2) * 0.75 + 0.25) * MAX_SPEED;

    // If translation speed is 0, make it slightly larger so the wheels will still
    // turn
    if (translationSpeed == 0.0) {
      translationSpeed = 0.001;
    }

    // Set the translation velocity vector
    chassis.setTranslationVelocity(Vector2D.createPolarCoordinates(translationSpeed, this.translationAngle));

    // Set the rotation velocity
    if (Math.abs(gamepad.getRawAxis(4)) > 0.1) {
      chassis.setRotationVelocity(-(gamepad.getRawAxis(4) * 0.9 + 0.1) * MAX_ROTATION);
    } else {
      chassis.setRotationVelocity(0);
    }

    // Update the chassis
    chassis.update();
  }

  @Override
  public void testInit() {
    disableCalibrateMode();
  }

  @Override
  public void testPeriodic() {
  }

  private void enableCalibrateMode() {
    if (!calibrateMode) {
      calibrateMode = true;

      // Set all motors to coast mode
      for (Map.Entry<String, PIDNeo> neo : neos.entrySet()) {
        neo.getValue().setIdleMode(IdleMode.kCoast);
      }
    }
  }

  private void disableCalibrateMode() {
    if (calibrateMode) {
      calibrateMode = false;

      // Set all motors back to brake mode
      for (Map.Entry<String, PIDNeo> neo : neos.entrySet()) {
        neo.getValue().setIdleMode(IdleMode.kBrake);
      }
      // Set azimuths to 0
      transmissions.get("FL Azimuth").calibratePosition(0);
      transmissions.get("BL Azimuth").calibratePosition(0);
      transmissions.get("BR Azimuth").calibratePosition(0);
      transmissions.get("FR Azimuth").calibratePosition(0);

      // Set gyro to 0
      navx.calibrateYaw(0);
    }
  }
}
