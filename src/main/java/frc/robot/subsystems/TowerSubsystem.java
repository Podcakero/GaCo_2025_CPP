// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Inches;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class TowerSubsystem extends SubsystemBase {

	private TowerState currentState = TowerState.INIT;

	private Timer stateTimer = new Timer();

	private ElevatorSubsystem elevator;
	private WristSubsystem wrist;
	private TowerEvent pendingEvent = TowerEvent.NONE;   
	private double safetyFactor = 1;
	private int currentLevel = 0;

	/** Creates a new Tower. */
	public TowerSubsystem(ElevatorSubsystem elevator, WristSubsystem wrist) {
		this.elevator = elevator;
		this.wrist = wrist;
		stateTimer.start();
	}

	public void initialize() {
		setState(TowerState.INIT);
		pendingEvent = TowerEvent.NONE;
		wrist.initialize();
		elevator.initialize();
	}

	public void resetFrameRate() {
		elevator.resetFrameRate();
		wrist.resetFrameRate();
	}

	@Override
	public void periodic() {
		// This method will be called once per scheduler run
		runStateMachine();
		updateDashboard();
	}

	public void homeTower() {
		elevator.resetElevatorControl();
		wrist.resetWristControl();
		setState(TowerState.INIT);
	}
	

	public void runStateMachine() {
		switch(currentState){
			case INIT: {
				if (elevator.getHeight().lt(Constants.ElevatorConstants.kSafeHomeHeight)) {
					wrist.setGoalAngle(Constants.WristConstants.kIntakeAngle);
					elevator.setGoalPosition(Constants.ElevatorConstants.kIntakeHeight);	
					setState(TowerState.HOMING);
				} else {
					wrist.setGoalAngle(Constants.WristConstants.kSafeAngle);
					setState(TowerState.PREPARING_TO_HOME);
				}
				break;
			}

			case PREPARING_TO_HOME: {
				if (wrist.inPosition()) {
					elevator.setGoalPosition(Constants.ElevatorConstants.kIntakeHeight);	
					setState(TowerState.LOWERING_TO_HOME);
				}
				break;
			}

			case LOWERING_TO_HOME: {
				if (elevator.inPosition()) {
					wrist.setGoalAngle(Constants.WristConstants.kIntakeAngle);
					setState(TowerState.HOMING);
				}
				break;
			}

			case HOMING: {
				if (wrist.inPosition() && elevator.inPosition()){
					setState(TowerState.HOME);
				}
				break;
			}

			
			case HOME: {
				if (isTriggered(TowerEvent.INTAKE_CORAL) || isHoldingGoTo()){
					wrist.setIntakeSpeed(Constants.WristConstants.kCoralIntakePower);
					setState(TowerState.INTAKING);
				} else if (isTriggered(TowerEvent.INTAKE_ALGAE)){
					wrist.setGoalAngle(Constants.WristConstants.kSafeAngle);
					setState(TowerState.FLIPING_WRIST_TO_STAFE);
				} else {
					currentLevel = 0;
				}
				break;
			}

			case FLIPING_WRIST_TO_STAFE: {
				if(wrist.inPosition()){
					elevator.setGoalPosition(Constants.ElevatorConstants.kAlgaeHighHight);
					setState(TowerState.RAISING_LIFT_TO_ALGAE_HIGH);
				}
				break;
			}

			case RAISING_LIFT_TO_ALGAE_HIGH: {
				if(elevator.inPosition()){
					wrist.setGoalAngle(Constants.WristConstants.kAlgaeIntakeAngle);
					setState(TowerState.FLIPING_WRIST_TO_ALGAE_INTAKE);
				}
				break;
			}

			case FLIPING_WRIST_TO_ALGAE_INTAKE: {
				if(wrist.inPosition()){
					wrist.setIntakeSpeed(Constants.WristConstants.kAlgaeIntakePower);
					setState(TowerState.WAITING_FOR_ALGAE);
				}
				break;
			}

			case WAITING_FOR_ALGAE: {
		
				if(isTriggered(TowerEvent.INTAKE_ALGAE)){
					wrist.setIntakeSpeed(Constants.WristConstants.kCoralIntakePower);  // Score Algae
					setState(TowerState.PAUSING);
				} else if (isTriggered(TowerEvent.GOTO_L1)) {
					currentLevel = 1;
					elevator.setGoalPosition(Constants.ElevatorConstants.kL1Height);
					wrist.setGoalAngle(Constants.WristConstants.kAlgaeIntakeAngle);
					setState(TowerState.CHANGING_ALGAE_HEIGHT);
				} else if (isTriggered(TowerEvent.GOTO_L4)) {
					currentLevel = 4;
					elevator.setGoalPosition(Constants.ElevatorConstants.kL4Height);
					wrist.setGoalAngle(Constants.WristConstants.kHighAlgaeAngle);
					setState(TowerState.CHANGING_ALGAE_HEIGHT);
				} else if(wrist.getIntakeCurrent() > 40){
					wrist.setIntakeSpeed(Constants.WristConstants.kLowAlgaeIntakePower);  // hold lightly
				}
				break;
			}

			case CHANGING_ALGAE_HEIGHT: {
				if (elevator.inPosition()) {
					setState(TowerState.WAITING_FOR_ALGAE);
				}
				break;
			}

			case INTAKING: {
				if (wrist.gotEnterCoral()){
					wrist.setGoalAngle(Constants.WristConstants.kSafeAngle);
					wrist.setIntakeSpeed(Constants.WristConstants.kCoralSlowIntakePower);
					setState(TowerState.GOING_TO_SAFE);
				}
				break;
			}
			
			case GOING_TO_SAFE: {
				if (wrist.inPosition()){
					wrist.setIntakeSpeed(0);
					setState(TowerState.GOT_CORAL);
				} else {
					if (wrist.gotExitCoral()) {
						wrist.setIntakeSpeed(0);
					}
				}
				break;
			}

			case GOT_CORAL: {
				if (isTriggered(TowerEvent.GOTO_L1)){
					elevator.setGoalPosition(Constants.ElevatorConstants.kL1Height);
					currentLevel = 1;
					wrist.setIntakeSpeed(0);
					setState(TowerState.RAISING);
				} else if (isTriggered(TowerEvent.GOTO_L2)){
					elevator.setGoalPosition(Constants.ElevatorConstants.kL2Height);
					currentLevel = 2;
					wrist.setIntakeSpeed(0);
					setState(TowerState.RAISING);
				} else if (isTriggered(TowerEvent.GOTO_L3)) {
					elevator.setGoalPosition(Constants.ElevatorConstants.kL3Height);
					currentLevel = 3;
					wrist.setIntakeSpeed(0);
					setState(TowerState.RAISING);
				} else if (isTriggered(TowerEvent.GOTO_L4)){
					elevator.setGoalPosition(Constants.ElevatorConstants.kL4Height);
					currentLevel = 4;
					wrist.setIntakeSpeed(0);
					setState(TowerState.RAISING_TO_L4);
				} else {
					if (wrist.gotExitCoral()) {
						wrist.setIntakeSpeed(0);
					}
				}
				break;
			}
		

			case RAISING: {
				if (elevator.inPosition()) {
					setState(TowerState.READY_TO_SCORE);
				}
				break;
			}

			case RAISING_TO_L4: {
				if (elevator.inPosition()) {
					wrist.setGoalAngle(Constants.WristConstants.kL4Angle);
					setState(TowerState.TILTING_TO_SCORE_L4);
				}
				break;
			}

			case TILTING_TO_SCORE_L4: {
				if (wrist.inPosition()) {
					setState(TowerState.READY_TO_SCORE);
				}
				break;
			}

			case READY_TO_SCORE: {
				if (isTriggered(TowerEvent.SCORE_CORAL)) {
					wrist.setIntakeSpeed(Constants.WristConstants.kCoralScoringPower);
					setState(TowerState.SCORING_CORAL);
				} else if ((isTriggered(TowerEvent.GOTO_L4))  && (currentLevel != 4)){
					elevator.setGoalPosition(Constants.ElevatorConstants.kL4Height);
					currentLevel = 4;
					setState(TowerState.RAISING_TO_L4);
				} else if ((isTriggered(TowerEvent.GOTO_L3))  && (currentLevel != 3)){
					elevator.setGoalPosition(Constants.ElevatorConstants.kL3Height);
					wrist.setGoalAngle(Constants.WristConstants.kSafeAngle);
					currentLevel = 3;
					setState(TowerState.RAISING);
				} else if ((isTriggered(TowerEvent.GOTO_L2))  && (currentLevel != 2)){
					elevator.setGoalPosition(Constants.ElevatorConstants.kL2Height);
					wrist.setGoalAngle(Constants.WristConstants.kSafeAngle);
					currentLevel = 2;
					setState(TowerState.RAISING);
				} else if ((isTriggered(TowerEvent.GOTO_L1))  && (currentLevel != 1)){
					elevator.setGoalPosition(Constants.ElevatorConstants.kL1Height);
					wrist.setGoalAngle(Constants.WristConstants.kSafeAngle);
					currentLevel = 1;
					setState(TowerState.RAISING);
				}
				break;
			}

			case SCORING_CORAL: {
				if (!wrist.gotExitCoral()) {
					setState(TowerState.PAUSING);
				} else if ( stateTimer.hasElapsed(0.3)) {
					wrist.setGoalAngle(Constants.WristConstants.kSafeAngle);
					setState(TowerState.PAUSING);
				}
				break;
			}

			case PAUSING: {
				if (stateTimer.hasElapsed(0.25)) {
					wrist.setIntakeSpeed(0);
					wrist.setGoalAngle(Constants.WristConstants.kSafeAngle);
					elevator.setGoalPosition(Constants.ElevatorConstants.kIntakeHeight);
					setState(TowerState.LOWERING);
				}
				break;
			}

			case LOWERING: {
				if (elevator.inPosition()){
					wrist.setGoalAngle(Constants.WristConstants.kIntakeAngle);
					setState(TowerState.GOING_TO_INTAKE);
				}
				break;
			}
			
			case GOING_TO_INTAKE: {
				if (wrist.inPosition()){
					setState(TowerState.HOME);
				}
			}
		}
	}

	public double getTowerSpeedSafetyFactor() {
		//  Determine what portion of full speed can be used based on the tower State
		safetyFactor = 1;

		if ((currentState == TowerState.SCORING_CORAL) || (currentState == TowerState.PAUSING)) {
			safetyFactor = 0.25;
		} else if (elevator.getHeight().gt(Constants.ElevatorConstants.kElevatorSpeedSafeHeight)) {
			double span    = Constants.ElevatorConstants.kElevatorMaxHeight.in(Inches) - Constants.ElevatorConstants.kElevatorSpeedSafeHeight.in(Inches); 
			double overage = elevator.getHeight().in(Inches) - Constants.ElevatorConstants.kElevatorSpeedSafeHeight.in(Inches); 
			double ratio   = overage / span;
			safetyFactor   = 1.0 - (0.5 * ratio) ;
		}

		return safetyFactor;
	}

	public void triggerEvent(TowerEvent event){
		pendingEvent = event;
	}

	public TowerEvent getPendingEvent() {
		return pendingEvent;
	}
	
	public TowerState getState() {
		return currentState;
	}


	// -- Private Methods  ----------------------------------------------------

	
	private void updateDashboard() {
		SmartDashboard.putString("Tower State", currentState.toString() + " <- " + pendingEvent.toString());
		SmartDashboard.putNumber("Safety Factor", safetyFactor * 100);
	}
	
	private Boolean isTriggered(TowerEvent event){
		if (pendingEvent == event) {
			pendingEvent = TowerEvent.NONE;
			return true;
		} else {
			return false;
		}
	}

	private Boolean isHoldingGoTo(){
		if ((pendingEvent == TowerEvent.GOTO_L1) || (pendingEvent == TowerEvent.GOTO_L2) || 
		    (pendingEvent == TowerEvent.GOTO_L3) || (pendingEvent == TowerEvent.GOTO_L4)) {
			return true;
		} else {
			return false;
		}
	}


	private void setState(TowerState newState){
		currentState = newState;
		stateTimer.reset();
	}
}
