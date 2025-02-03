// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.pathplanner.lib.config.PIDConstants;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Tower extends SubsystemBase {
  /** Creates a new Tower. */
  public Tower() {}

  // Declare Constants
  final double FIN_KP = 1.0;
  final double FIN_KI = 0.0;
  final double FIN_KD = 0.0;
  final double LIFT_KP = 1.0;
  final double LIFT_KI = 0.0;
  final double LIFT_KD = 0.0;



  // Declare class members
  public double currentFinAngle;
  public double currentLiftHeight;
  public boolean collectingCoral;
  public double finTakeSpeed;
  public double finAngleSetPoint;
  public double liftHeightSetPoint;
  public double finPower;
  public double liftPower;
  public TowerState currentTowerState = TowerState.HOME;

  private PIDController finController;
  private PIDController liftController;

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    readSensor();
    runLiftControl();
    runFinControl();
    runStateMachine();
    updateDashboard();
  }

  public void initialize() {
    finController = new PIDController(FIN_KP, FIN_KI, FIN_KD);
    liftController = new PIDController(LIFT_KP, LIFT_KI, LIFT_KD);
  }

  public void readSensor() {

  }

  public void runStateMachine() {
    switch (currentTowerState) {
      case HOME:
        
        break;

      case LIFTING:

        break;

    
    
      default:
        break;
    }

  }

  public void runLiftControl() {
    liftPower = liftController.calculate(currentLiftHeight, liftHeightSetPoint);
  }

  public void runFinControl(){
    finPower = finController.calculate(currentFinAngle, finAngleSetPoint);
  }

  private void updateDashboard() {
    SmartDashboard.putNumber("Current Fin Angle", currentFinAngle);
    SmartDashboard.putNumber("Current Lift Height", currentLiftHeight);
    SmartDashboard.putNumber("Fin Angle Set Point", finAngleSetPoint);
    SmartDashboard.putNumber("Lift Height Set Point", liftHeightSetPoint);
  }

}
