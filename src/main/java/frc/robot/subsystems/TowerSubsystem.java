// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

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

	/** Creates a new Tower. */
	public TowerSubsystem(ElevatorSubsystem elevator, WristSubsystem wrist) {
		this.elevator = elevator;
		this.wrist = wrist;
		stateTimer.start();
	}

	public void initialize() {
		setState(TowerState.INIT);
		pendingEvent = TowerEvent.NONE;
	}

	@Override
	public void periodic() {
		// This method will be called once per scheduler run
		runStateMachine();
		updateDashboard();
	}

	public void homeTower() {
		wrist.setGoalAngle(Constants.WristConstants.kIntakeAngle);
		elevator.setGoalPosition(Constants.ElevatorConstants.kIntakeHeight);
	}
	

	public void runStateMachine() {
		switch(currentState){
			case INIT: {
				homeTower();
				setState(TowerState.HOMING);
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
				}
				break;
			}

			case INTAKING: {
				if (wrist.gotCoral()){
					wrist.setIntakeSpeed(0);
					wrist.setGoalAngle(Constants.WristConstants.kSafeAngle);
					setState(TowerState.GOING_TO_SAFE);
				}
				break;
			}

			case GOING_TO_SAFE: {
				if (wrist.inPosition()){
					setState(TowerState.GOT_CORAL);
				}
				break;
			}

			case GOT_CORAL: {
				if (isTriggered(TowerEvent.GOTO_L3)) {
					elevator.setGoalPosition(Constants.ElevatorConstants.kL3Height);
					setState(TowerState.RAISING);
				} else if (isTriggered(TowerEvent.GOTO_L2)){
					elevator.setGoalPosition(Constants.ElevatorConstants.kL2Height);
					setState(TowerState.RAISING);
				} else if (isTriggered(TowerEvent.GOTO_L1)){
					elevator.setGoalPosition(Constants.ElevatorConstants.kL1Height);
					setState(TowerState.RAISING);
				} else if (isTriggered(TowerEvent.GOTO_L4)){
					elevator.setGoalPosition(Constants.ElevatorConstants.kL4Height);
					setState(TowerState.RAISING_TO_L4);
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
				}
				break;
			}

			case SCORING_CORAL: {
				if (!wrist.gotCoral()) {
					wrist.setIntakeSpeed(0);
					setState(TowerState.PAUSING);
				}
				break;
			}

			case PAUSING: {
				if (stateTimer.hasElapsed(0.5)) {
					wrist.setGoalAngle(Constants.WristConstants.kSafeAngle);
					elevator.setGoalPosition(Constants.ElevatorConstants.kIntakeHeight);
					setState(TowerState.LOWERING);
				}
				break;
			}

			case LOWERING: {
				if (elevator.inPosition()){
					wrist.setGoalAngle(Constants.WristConstants.kIntakeAngle);
					// wrist.setIntakeSpeed(Constants.WristConstants.kCoralIntakePower);
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
