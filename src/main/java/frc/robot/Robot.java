/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import com.ctre.phoenix.CANifier;
import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.PowerDistributionPanel;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.I2C.Port;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;



public class Robot extends TimedRobot {

  PowerDistributionPanel pdp;
  AHRS navx;

  Module[] modules;

  NetworkTableEntry entryYaw;

  @Override
  public void robotInit() {
    pdp = new PowerDistributionPanel(20);
    navx = new AHRS(Port.kOnboard);

    modules = new Module[4];
    
    modules[0] = new Module("FL",
        new Motor("FL+", new CANSparkMax(16, MotorType.kBrushless), ()->pdp.getCurrent(0)),
        new Motor("FL-", new CANSparkMax(1, MotorType.kBrushless), ()->pdp.getCurrent(1)));

    modules[1] = new Module("BL",
        new Motor("BL+", new CANSparkMax(2, MotorType.kBrushless), ()->pdp.getCurrent(2)),
        new Motor("BL-", new CANSparkMax(3, MotorType.kBrushless), ()->pdp.getCurrent(3)));

    modules[2] = new Module("BR",
        new Motor("BR+", new CANSparkMax(12, MotorType.kBrushless), ()->pdp.getCurrent(12)),
        new Motor("BR-", new CANSparkMax(13, MotorType.kBrushless), ()->pdp.getCurrent(13)));

    modules[3] = new Module("FR",
        new Motor("FR+", new CANSparkMax(14, MotorType.kBrushless), ()->pdp.getCurrent(14)),
        new Motor("FR-", new CANSparkMax(15, MotorType.kBrushless), ()->pdp.getCurrent(15)));

    entryYaw = SmartDashboard.getEntry("Yaw");
    entryYaw.setDouble(0);

  }

  @Override
  public void robotPeriodic() {
    for (Module mod : modules) {
      mod.update();
    }
    entryYaw.setDouble(navx.getYaw());
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
  }

  @Override
  public void testInit() {
  }

  @Override
  public void testPeriodic() {
  }

}
