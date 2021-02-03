/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.RobotMap;

/**
 * The Intake delivers "Power Cells" to this subsystem to be transfered to the
 * Shooter. This subsystem also "indexes" said "Power Cells" for rapid shooting
 */
public class Hopper extends SubsystemBase {

  public VictorSPX kickerWheel = new VictorSPX(RobotMap.kickerWheel);
  public TalonSRX backFeederBelt = new TalonSRX(RobotMap.backFeederBelt);
  public TalonSRX frontFeederBeltWheel = new TalonSRX(RobotMap.fronFeederBeltWheel);

  public DigitalInput topBeam = new DigitalInput(RobotMap.topBeam);
  public DigitalInput lowBeam = new DigitalInput(RobotMap.lowBeam);

  public enum HopperControlMode {
    IDLE,
    INTAKING,
    FEEDING,
    UNJAMMING
  }

  public HopperControlMode hopperControlMode = HopperControlMode.IDLE;

  public synchronized HopperControlMode getControlMode() {
    return hopperControlMode;
}

  public synchronized void setControlMode(HopperControlMode controlMode) {
    this.hopperControlMode = controlMode;
  }

  public boolean getTopBeam() {
    return !topBeam.get();
  }

  public boolean getLowBeam() {
    return !lowBeam.get();
  }

  public Hopper() {
    backFeederBelt.setInverted(true);
    frontFeederBeltWheel.setInverted(false);

    backFeederBelt.setNeutralMode(NeutralMode.Coast);
    frontFeederBeltWheel.setNeutralMode(NeutralMode.Coast);
    kickerWheel.setNeutralMode(NeutralMode.Brake);
  }

  public void setBackBeltSpeed(double percentOutput) {
    backFeederBelt.set(ControlMode.PercentOutput, percentOutput);
  }

  public void setKickerSpeed(double percentOutput) {
    kickerWheel.set(ControlMode.PercentOutput, percentOutput);
  }

  public void setFeederWheelFrontBeltSpeed(double percentOutput) {
    frontFeederBeltWheel.set(ControlMode.PercentOutput, percentOutput);
  }

  public void setHopperSpeed(double kickerSpeed, double backBeltSpeed, double frontBeltWheelSpeed) {
    kickerWheel.set(ControlMode.PercentOutput, kickerSpeed);
    backFeederBelt.set(ControlMode.PercentOutput, backBeltSpeed);
    frontFeederBeltWheel.set(ControlMode.PercentOutput, frontBeltWheelSpeed);
  }

  public void stopSystems() {
    kickerWheel.set(ControlMode.PercentOutput, 0);
    backFeederBelt.set(ControlMode.PercentOutput, 0);
    frontFeederBeltWheel.set(ControlMode.PercentOutput, 0);
  }

  @Override
  public void periodic() {
    SmartDashboard.putBoolean("Top Beam", getTopBeam());
    SmartDashboard.putBoolean("Low Beam", getLowBeam());
    synchronized (Hopper.this) {
      switch (getControlMode()) {
        case IDLE:
          stopSystems();
          break;
        case INTAKING:
          if (!getTopBeam() && !getLowBeam()) {
            setHopperSpeed(.2, .4, .4);
          }
          else if (getTopBeam() && !getLowBeam()) {
            setKickerSpeed(.2);
            setBackBeltSpeed(0);
            setFeederWheelFrontBeltSpeed(.5);
          }
          else if (getTopBeam() && getLowBeam()){
            setHopperSpeed(0, 0, 0);
          }
          break;
        case FEEDING:
            if (RobotContainer.shooter.isFar) {
            setHopperSpeed(.2, .28, .28);
            }
            else {
              setHopperSpeed(.2, .5, .5);
            }
          break;
        case UNJAMMING:
          setHopperSpeed(-.4, -.75, -.75);
          break;
      }
    }
  }
}
