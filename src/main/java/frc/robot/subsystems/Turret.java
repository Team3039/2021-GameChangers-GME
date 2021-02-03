/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import static frc.robot.Constants.TURRET_PPR_TO_DEGREES;
import static frc.robot.Constants.kP_TURRET;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.RobotMap;

/**
 * This device is responisble for the rotational control of the "Shooter" and the tracking of the 
 * retro-reflective target on the "Power Port"
 */
public class Turret extends SubsystemBase {
  
  public TalonSRX turret = new TalonSRX(RobotMap.turret);
  public DigitalInput turretSwitch = new DigitalInput(RobotMap.turretSwitch);

  public static double startPos = -90;
  
  public Turret() {
    turret.setSelectedSensorPosition(0);
    turret.configSelectedFeedbackCoefficient(TURRET_PPR_TO_DEGREES); //Convert to Degrees of Revolution
    turret.setSensorPhase(true);
    setLed(false);
    setPipeline(0);
    setCamMode(false);
    turret.config_kP(0, 0.05);
    turret.selectProfileSlot(0, 0);
  }

  public enum TurretControlMode {
    DRIVER, 
    TRACKING,
    JOYSTICK,
    CLIMB
  }

  public TurretControlMode turretControlMode = TurretControlMode.DRIVER;

  public synchronized TurretControlMode getControlMode() {
    return turretControlMode;
  }

  public synchronized void setControlMode(TurretControlMode controlMode) {
    this.turretControlMode = controlMode;
  }

  public void setLed(boolean isOn) {
    if (isOn) {
      NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(3); //Force LED on
    }
    else {
      NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(1); //Force LED off
    }
  }

  public void setPipeline(int pipeline) {
    NetworkTableInstance.getDefault().getTable("limelight").getEntry("pipeline").setNumber(pipeline); //Set the Pipeline #
  }

  public void setCamMode(boolean isTracking) {
    if (isTracking) {
      NetworkTableInstance.getDefault().getTable("limelight").getEntry("camMode").setNumber(0);
    }
    else {
      NetworkTableInstance.getDefault().getTable("limelight").getEntry("camMode").setNumber(1);
    }
  }

  public void trackTarget() {
    double errorX = (0 - getTargetX()) * kP_TURRET;

    if (getTurretSwitch() && getTurretPosition() > 245) {
      turret.set(ControlMode.PercentOutput, -.1);
    }
    else if (getTurretSwitch() && getTurretPosition() < -110) {
      turret.set(ControlMode.PercentOutput, .1);
    }
    else {
      turret.set(ControlMode.PercentOutput, errorX);
    }
  }
  
  public void resetPose() {
    double errorX = (getTurretPosition()) * kP_TURRET;

    if (getTurretSwitch() && getTurretPosition() > 245) {
      turret.set(ControlMode.PercentOutput, -.1);
    }
    else if (getTurretSwitch() && getTurretPosition() < -110) {
      turret.set(ControlMode.PercentOutput, .1);
    }
    else {
      turret.set(ControlMode.PercentOutput, errorX);
    }
  }

  public double getTurretPosition() {
    return getCurrentPosition() % 360;
  }

  public void setTrackingMode() {
    setPipeline(0);
    setLed(true);
    setCamMode(true);
  }

  public void setDriverCamMode() {
    NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(1); //Turns LED off
    NetworkTableInstance.getDefault().getTable("limelight").getEntry("camMode").setNumber(1); //Disable Vision Processing and Doubles Exposure
  }

  public double convertAngle(double angle) {
    if (angle >= 0) {
        return angle;
    }
    else {
        return angle + 360;
    }
}

  public void resetTurretPosition() {
    turret.setSelectedSensorPosition(0);
  }

  public void setTurretPosition(double degrees) {
    turret.config_kP(0, 0.0001);
    double modDegrees = degrees % 360;
    turret.set(ControlMode.Position, modDegrees);
  }

  public double getTargetX() {
    return Robot.targetX; 
  }

  public double getTargetY() {
    return Robot.targetY;
  }

  public double getTargetArea() {
    return Robot.targetArea;
  }

  public double getErrorX() {
    return getTargetX() - getCurrentPosition();
  }

  public boolean hasTarget() {
    if(Robot.targetValid == 1) {
      return true;
    }
    else {
      return false;
    }
  }

  public void manualControl() {
    if (getTurretSwitch() && getTurretPosition() > 245) {
      turret.set(ControlMode.PercentOutput, -.1);
    }
    else if (getTurretSwitch() && getTurretPosition() < -110) {
      turret.set(ControlMode.PercentOutput, .1);
    }
    else {
      turret.set(ControlMode.PercentOutput, RobotContainer.getOperator().getLeftXAxis());
    }
  }
  
  public void turretReverse() {
    double errorX = (getTurretPosition() - 180) * kP_TURRET;
    if (getTurretSwitch() && getTurretPosition() > 245) {
      turret.set(ControlMode.PercentOutput, -.1);
    }
    else if (getTurretSwitch() && getTurretPosition() < -110) {
      turret.set(ControlMode.PercentOutput, .1);
    }
    else {
      turret.set(ControlMode.PercentOutput, errorX);
    }
  }

  public Boolean getTurretSwitch() {
    return !turretSwitch.get();
  }

  public double getCurrentPosition() {
    return turret.getSelectedSensorPosition();
  }

  @Override
  public void periodic() {
    synchronized (Turret.this) {
      switch (getControlMode()) {
        case DRIVER:
          setDriverCamMode();
          resetPose();
          break;
        case TRACKING:
          setTrackingMode();
          trackTarget();
          break;
        case JOYSTICK:
          manualControl();
          break;
        case CLIMB:
          turretReverse();
          break;
        default:
        setDriverCamMode();
        resetPose();
        break;
      }
    }
  }
}
