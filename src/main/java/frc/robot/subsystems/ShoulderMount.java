// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.sim.TalonFXSimState;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ElectronicsIDs;
import frc.robot.Constants.FloorIntakeConstants;
import frc.robot.Constants.ShoulderMountConstants;
import frc.robot.sim.PhysicsSimFX;
import org.littletonrobotics.junction.Logger;
import edu.wpi.first.math.util.Units;

public class ShoulderMount extends SubsystemBase {

  TalonFX shoulderMotor;
  private final TalonFXSimState shoulderMotorSim;
  private final DCMotorSim shoulderMotorModel =
      new DCMotorSim(
          edu.wpi.first.math.system.plant.DCMotor.getKrakenX60(1), 1, Constants.jKgMetersSquared);

  private final VelocityVoltage voltageVelocity =
      new VelocityVoltage(0, 0, true, 0, 0, false, false, false);
  private final NeutralOut brake = new NeutralOut();

  boolean simulationInitialized = false;
      
  public ShoulderMount() {
    shoulderMotor = new TalonFX(ElectronicsIDs.ShoulderMotorID);
    applyMotorConfigs(InvertedValue.Clockwise_Positive);

    shoulderMotorSim = shoulderMotor.getSimState();
  }
  @Override
  public void periodic() {
    logData();
  }

  public void start() {
    Logger.recordOutput("ShoulderMount/RPMDesired", ShoulderMountConstants.MotorRPM);
    // put code here
  }
  public void setPosition(Constants.IntakeState intakeState) {
    if (Constants.IntakeState.BASE == intakeState && getState() == Constants.IntakeState.INTAKING) { // set  position to base
        final MotionMagicVoltage request = new MotionMagicVoltage(Units.degreesToRotations(Constants.ShoulderMountConstants.baseAngle));
        shoulderMotor.setControl(request);
    }
    else if (Constants.IntakeState.BASE == intakeState && getState() == Constants.IntakeState.INTAKING) {
        final MotionMagicVoltage request = new MotionMagicVoltage(Units.degreesToRotations(Constants.ShoulderMountConstants.intakeAngle));
        shoulderMotor.setControl(request);
    }
  }

  public Constants.IntakeState getState() {
    
  }

  public void reverse() {
    Logger.recordOutput("ShoulderMount/RPMDesired", -ShoulderMountConstants.MotorRPM);
    // put code here
  }

  

  public void stop() {
    shoulderMotor.setControl(brake);
  }

/* SIMULATION */
    
/* LOGGING */

    private void logData() {
        Logger.recordOutput("ShoulderMount/RPMActual", shoulderMotor.getVelocity().getValue() * 60);
        Logger.recordOutput("ShoulderMount/CurrentSupply", shoulderMotor.getSupplyCurrent().getValue());
    }

 /* CONFIG */

    private void applyMotorConfigs(InvertedValue inverted) {
        // set PID Values
        TalonFXConfiguration motorConfigs = new TalonFXConfiguration();
        motorConfigs.Slot0.kV = FloorIntakeConstants.FF;
        motorConfigs.Slot0.kP = FloorIntakeConstants.KP;
        motorConfigs.Slot0.kI = FloorIntakeConstants.KI;
        motorConfigs.Slot0.kD = FloorIntakeConstants.KD;

        MotorOutputConfigs invertConfigs = new MotorOutputConfigs();
        invertConfigs.Inverted = inverted;

        // set current limit
        CurrentLimitsConfigs currentLimitConfigs = motorConfigs.CurrentLimits;
        currentLimitConfigs.SupplyCurrentLimit = ShoulderMountConstants.SupplyCurrentLimit;
        currentLimitConfigs.SupplyCurrentLimitEnable = true; // Start with stator limits off

        StatusCode status = StatusCode.StatusCodeNotInitialized;

        // Try five times to apply the Axis motor config
        for (int i = 0; i < 5; ++i) {
            status = shoulderMotor.getConfigurator().apply(motorConfigs, 0.05);
            if (status.isOK())
                break;
        }
        if (!status.isOK()) {
            System.out.println(
                    "Could not apply motor output configs to intake motor, with error code: " + status.toString());
        }

        // Try five times to apply the Axis motor invert config
        for (int i = 0; i < 5; ++i) {
            status = shoulderMotor.getConfigurator().apply(invertConfigs, 0.05);
            if (status.isOK())
                break;
        }
        if (!status.isOK()) {
            System.out.println(
                    "Could not apply invert configs to intake motor, with error code: " + status.toString());
        }

        // Try five times to apply the Axis motor current config
        for (int i = 0; i < 5; ++i) {
            status = shoulderMotor.getConfigurator().apply(currentLimitConfigs, 0.05);
            if (status.isOK())
                break;
        }
        if (!status.isOK()) {
            System.out.println(
                    "Could not apply current limit configs to intake motor, with error code: " + status.toString());
        }
    }

    /* SIMULATION */

    private void simulationInit() {
        PhysicsSimFX.getInstance().addTalonFX(shoulderMotor, 0.001);
    }

    @Override
    public void simulationPeriodic() {
        if (!simulationInitialized) {
            simulationInit();
            simulationInitialized = true;
        }

        shoulderMotorSim.setSupplyVoltage(RobotController.getBatteryVoltage());
        double axisVoltage = shoulderMotorSim.getMotorVoltage();
        shoulderMotorModel.setInputVoltage(axisVoltage);
        shoulderMotorModel.update(0.02);
        shoulderMotorSim.setRotorVelocity(shoulderMotorModel.getAngularVelocityRPM() / 60.0);
        shoulderMotorSim.setRawRotorPosition(shoulderMotorModel.getAngularPositionRotations());
    }
  
}
