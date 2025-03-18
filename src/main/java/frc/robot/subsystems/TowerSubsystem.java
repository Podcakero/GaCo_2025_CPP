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
	private boolean goDirectAlgae = false;

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

	public void enableGoToDirectAlgae(){
		goDirectAlgae = true;
	}

	@Override
	public void periodic() {
		// This method will be called once per scheduler run
		runStateMachine();
		updateDashboard();
	}

	public void homeTower() {
		wrist.setIntakeSpeed(0);
		elevator.resetElevatorControl();
		wrist.resetWristControl();
		pendingEvent = TowerEvent.NONE;
		setState(TowerState.INIT);
	}
	

	public void runStateMachine() {
		switch(currentState){

			// =================== Initializing ===================
			case INIT: {
				if (elevator.getHeight().lt(Constants.Elevator.kSafeHomeHeight)) {
					wrist.setGoalAngle(Constants.Wrist.kIntakeAngle);
					elevator.setGoalPosition(Constants.Elevator.kIntakeHeight);	
					setState(TowerState.HOMING);
				} else {
					wrist.setGoalAngle(Constants.Wrist.kSafeAngle);
					setState(TowerState.PREPARING_TO_HOME);
				}
				break;
			}

			case PREPARING_TO_HOME: {
				if (wrist.inPosition()) {
					elevator.setGoalPosition(Constants.Elevator.kIntakeHeight);	
					setState(TowerState.LOWERING_TO_HOME);
				}
				break;
			}

			case LOWERING_TO_HOME: {
				if (elevator.inPosition()) {
					wrist.setGoalAngle(Constants.Wrist.kIntakeAngle);
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

			// =================== GENERIC HOME condition ===================
			
			case HOME: {
				if (isTriggered(TowerEvent.INTAKE_CORAL) || isHoldingGoTo()){
					wrist.setIntakeSpeed(Constants.Wrist.kCoralIntakePower);
					setState(TowerState.INTAKING);
				} else if (isTriggered(TowerEvent.INTAKE_LOW_ALGAE)){
					wrist.setGoalAngle(Constants.Wrist.kSafeAngle);
					setState(TowerState.GET_SAFE_FOR_L2_ALGAE);
				} else if (isTriggered(TowerEvent.INTAKE_HIGH_ALGAE)){
					wrist.setGoalAngle(Constants.Wrist.kSafeAngle);
					setState(TowerState.GET_SAFE_FOR_L3_ALGAE);
				} else if (isTriggered(TowerEvent.SCORE)){
					wrist.setIntakeSpeed(Constants.Wrist.kCoralIntakePower);
					setState(TowerState.PAUSING_AFTER_SCORING_CORAL);
				} else {
					currentLevel = 0;
				}
				break;
			}

			// ====  CORAL specific states  ========================================

			case INTAKING: {
				if (wrist.gotEnterCoral()){
					wrist.setIntakeSpeed(Constants.Wrist.kCoralSlowIntakePower);
					setState(TowerState.INTAKE_PAUSE);
				}
				break;
			}

			case INTAKE_PAUSE: {
				if (wrist.gotExitCoral()) {
					wrist.setIntakeSpeed(0);
				}
				
				if (stateTimer.hasElapsed(0.2)) {
					wrist.setGoalAngle(Constants.Wrist.kSafeAngle);
					setState(TowerState.GOING_TO_SAFE);
				} 
				break;
			}
			
			case GOING_TO_SAFE: {
				if (wrist.inPosition()){
					wrist.setIntakeSpeed(0);
					setState(TowerState.GOT_CORAL);
				} else if (wrist.gotExitCoral()) {
						wrist.setIntakeSpeed(0);
				}
				break;
			}

			case GOT_CORAL: {
				if (isTriggered(TowerEvent.GOTO_L1)){
					elevator.setGoalPosition(Constants.Elevator.kL1CoralHeight);
					currentLevel = 1;
					wrist.setIntakeSpeed(0);
					setState(TowerState.RAISING_TO_L123);
				} else if (isTriggered(TowerEvent.GOTO_L2)){
					elevator.setGoalPosition(Constants.Elevator.kL2CoralHeight);
					currentLevel = 2;
					wrist.setIntakeSpeed(0);
					setState(TowerState.RAISING_TO_L123);
				} else if (isTriggered(TowerEvent.GOTO_L3)) {
					elevator.setGoalPosition(Constants.Elevator.kL3CoralHeight);
					currentLevel = 3;
					wrist.setIntakeSpeed(0);
					setState(TowerState.RAISING_TO_L123);
				} else if (isTriggered(TowerEvent.GOTO_L4)){
					elevator.setGoalPosition(Constants.Elevator.kL4CoralHeight);
					currentLevel = 4;
					wrist.setIntakeSpeed(0);
					setState(TowerState.RAISING_TO_L4);
				} else if (isTriggered(TowerEvent.SCORE)){
					wrist.setIntakeSpeed(Constants.Wrist.kCoralIntakePower);
					setState(TowerState.PAUSING_AFTER_SCORING_CORAL);
				} else {
					if (wrist.gotExitCoral()) {
						wrist.setIntakeSpeed(0);
					}
				}
				break;
			}
		

			case RAISING_TO_L123: {
				if (elevator.inPosition()) {
					setState(TowerState.READY_TO_SCORE_CORAL);
				}
				break;
			}

			case RAISING_TO_L4: {
				if (elevator.inPosition()) {
					wrist.setGoalAngle(Constants.Wrist.kL4Angle);
					setState(TowerState.TILTING_TO_SCORE_L4);
				}
				break;
			}

			case TILTING_TO_SCORE_L4: {
				if (wrist.inPosition()) {
					setState(TowerState.READY_TO_SCORE_CORAL);
				}
				break;
			}

			case READY_TO_SCORE_CORAL: {
				if (isTriggered(TowerEvent.SCORE)) {
					if (currentLevel >  1) {
						wrist.setIntakeSpeed(Constants.Wrist.kCoralL234ScoringPower);
					} else {
						wrist.setIntakeSpeed(Constants.Wrist.kCoralL1ScoringPower);
					}
					
					setState(TowerState.SCORING_CORAL);
				} else if ((isTriggered(TowerEvent.GOTO_L4))  && (currentLevel != 4)){
					elevator.setGoalPosition(Constants.Elevator.kL4CoralHeight);
					currentLevel = 4;
					setState(TowerState.RAISING_TO_L4);
				} else if ((isTriggered(TowerEvent.GOTO_L3))  && (currentLevel != 3)){
					elevator.setGoalPosition(Constants.Elevator.kL3CoralHeight);
					wrist.setGoalAngle(Constants.Wrist.kSafeAngle);
					currentLevel = 3;
					setState(TowerState.RAISING_TO_L123);
				} else if ((isTriggered(TowerEvent.GOTO_L2))  && (currentLevel != 2)){
					elevator.setGoalPosition(Constants.Elevator.kL2CoralHeight);
					wrist.setGoalAngle(Constants.Wrist.kSafeAngle);
					currentLevel = 2;
					setState(TowerState.RAISING_TO_L123);
				} else if ((isTriggered(TowerEvent.GOTO_L1))  && (currentLevel != 1)){
					elevator.setGoalPosition(Constants.Elevator.kL1CoralHeight);
					wrist.setGoalAngle(Constants.Wrist.kSafeAngle);
					currentLevel = 1;
					setState(TowerState.RAISING_TO_L123);
				}
				break;
			}

			case SCORING_CORAL: {
				if (!wrist.gotExitCoral() || stateTimer.hasElapsed(0.3)) {
					wrist.setGoalAngle(Constants.Wrist.kSafeAngle);
					setState(TowerState.PAUSING_AFTER_SCORING_CORAL);
				}
				break;
			}

			case PAUSING_AFTER_SCORING_CORAL: {
				if (wrist.inPosition() && stateTimer.hasElapsed(0.2)) {
					if (goDirectAlgae){  // special bypass to go to pickup Algae in Auto
						goDirectAlgae = false;  // reset the flag
						elevator.setGoalPosition(Constants.Elevator.kL2AlgaeHeight);
						wrist.setGoalAngle(Constants.Wrist.kAlgaeIntakeAngle);
						setState(TowerState.GOING_TO_ALGAE_INTAKE);
					} else {
						elevator.setGoalPosition(Constants.Elevator.kIntakeHeight);
						setState(TowerState.LOWERING);
					}
				}
				break;
			}

			case LOWERING: {
				if (elevator.inPosition()){
					wrist.setIntakeSpeed(0);
					wrist.setGoalAngle(Constants.Wrist.kIntakeAngle);
					setState(TowerState.GOING_TO_CORAL_INTAKE_ANGLE);
				}
				break;
			}
			
			case GOING_TO_CORAL_INTAKE_ANGLE: {
				if (wrist.inPosition()){
					setState(TowerState.HOME);
				}
				break;
			}

			// ====  ALGAE specific states  ========================================

			case GET_SAFE_FOR_L2_ALGAE: {
				if(wrist.inPosition()){
					elevator.setGoalPosition(Constants.Elevator.kL2AlgaeHeight);
					wrist.setGoalAngle(Constants.Wrist.kAlgaeIntakeAngle);
					setState(TowerState.GOING_TO_ALGAE_INTAKE);
				}
				break;
			}

			case GET_SAFE_FOR_L3_ALGAE: {
				if(wrist.inPosition()){
					elevator.setGoalPosition(Constants.Elevator.kL3AlgaeHeight);
					wrist.setGoalAngle(Constants.Wrist.kAlgaeIntakeAngle);
					setState(TowerState.GOING_TO_ALGAE_INTAKE);
				}
				break;
			}

			case GOING_TO_ALGAE_INTAKE: {
				if(elevator.inPosition() && wrist.inPosition()){
					wrist.setIntakeSpeed(Constants.Wrist.kAlgaeIntakePower);
					setState(TowerState.WAITING_FOR_ALGAE);
				}
				break;
			}

			case WAITING_FOR_ALGAE: {
				if(isTriggered(TowerEvent.SCORE)){
					if (currentLevel < 4) {
						wrist.setIntakeSpeed(Constants.Wrist.kCoralIntakePower);  // Score Algae
						Globals.GOT_ALGAE = false;
						setState(TowerState.PAUSING_AFTER_SCORING_ALGAE);
					} else {
						// Lobbing into Barge
						wrist.setGoalAngle(Constants.Wrist.kAlgaeReleaseAngle);
						setState(TowerState.WINDING_UP);
					}
				} else if (isTriggered(TowerEvent.GOTO_L1)) {
					currentLevel = 1;
					elevator.setGoalPosition(Constants.Elevator.kL1AlgaeHeight);
					wrist.setGoalAngle(Constants.Wrist.kAlgaeIntakeAngle);
					Globals.IDENTIFIED_TARGET = ApproachTarget.PROCESSOR;
					setState(TowerState.CHANGING_ALGAE_HEIGHT);
				} else if (isTriggered(TowerEvent.GOTO_L2)) {
					currentLevel = 2;
					elevator.setGoalPosition(Constants.Elevator.kL2AlgaeHeight);
					wrist.setGoalAngle(Constants.Wrist.kAlgaeIntakeAngle);
					setState(TowerState.CHANGING_ALGAE_HEIGHT);
				} else if (isTriggered(TowerEvent.GOTO_L3)) {
					currentLevel = 3;
					elevator.setGoalPosition(Constants.Elevator.kL3AlgaeHeight);
					wrist.setGoalAngle(Constants.Wrist.kAlgaeIntakeAngle);
					setState(TowerState.CHANGING_ALGAE_HEIGHT);
				} else if (isTriggered(TowerEvent.GOTO_L4)) {
					currentLevel = 4;
					elevator.setGoalPosition(Constants.Elevator.kL4AlgaeWindupHeight);
					wrist.setGoalAngle(Constants.Wrist.kAlgaeWindupAngle);
					setState(TowerState.CHANGING_ALGAE_HEIGHT);
				} else if(wrist.getIntakeCurrent() > 40){
					//wrist.setIntakeSpeed(Constants.WristConstants.kLowAlgaeIntakePower);  // hold lightly
					Globals.GOT_ALGAE = true;
				}
				break;
			}

			case CHANGING_ALGAE_HEIGHT: {
				if (elevator.inPosition()) {
					setState(TowerState.WAITING_FOR_ALGAE);
				}
				break;
			}

			case WINDING_UP: {
				if (wrist.inPosition()) {
					wrist.setIntakeSpeed(Constants.Wrist.kCoralL234ScoringPower);
					setState(TowerState.RELEASING)	;
				}
				break;
			}

			case RELEASING: {
				if (stateTimer.hasElapsed(2)){
					setState(TowerState.PAUSING_AFTER_SCORING_ALGAE);
				}
				break;
			}

			case PAUSING_AFTER_SCORING_ALGAE: {  // this state trigers Path Planner to move after scoring
				if (stateTimer.hasElapsed(0.3)){
					if(goDirectAlgae){  // special bypass to go to pickup Algae in Auto
						goDirectAlgae = false; //resets the flag
						wrist.setGoalAngle(Constants.Wrist.kAlgaeIntakeAngle);
						currentLevel = 1;
						wrist.setIntakeSpeed(Constants.Wrist.kAlgaeIntakePower);
						setState(TowerState.WAITING_FOR_ALGAE);
					} else {
						wrist.setGoalAngle(Constants.Wrist.kSafeAngle);
						elevator.setGoalPosition(Constants.Elevator.kIntakeHeight);
						setState(TowerState.LOWERING);
					}
				}
				break;
			}

		}
	}

	public double getTowerSpeedSafetyFactor() {
		//  Determine what portion of full speed can be used based on the tower State
		safetyFactor = 1;

		if ((currentState == TowerState.SCORING_CORAL) || (currentState == TowerState.PAUSING_AFTER_SCORING_CORAL)) {
			safetyFactor = 0.25;
		} else if (elevator.getHeight().gt(Constants.Elevator.kElevatorSpeedSafeHeight)) {
			double span    = Constants.Elevator.kElevatorMaxHeight.in(Inches) - Constants.Elevator.kElevatorSpeedSafeHeight.in(Inches); 
			double overage = elevator.getHeight().in(Inches) - Constants.Elevator.kElevatorSpeedSafeHeight.in(Inches); 
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
		SmartDashboard.putBoolean("l3Algae", goDirectAlgae);
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
