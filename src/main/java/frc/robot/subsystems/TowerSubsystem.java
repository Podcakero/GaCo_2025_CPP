// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class TowerSubsystem extends SubsystemBase {
	private TowerState currentState = TowerState.INIT;

	private final XboxController joystick = new XboxController(0);
	private Timer stateTimer = new Timer();

	private ElevatorSubsystem elevator;
	private WristSubsystem wrist;

	/** Creates a new Tower. */
	public TowerSubsystem(ElevatorSubsystem elevator, WristSubsystem wrist) {
		this.elevator = elevator;
		this.wrist = wrist;
		stateTimer.start();
	}

	public void initialize() {}

	@Override
	public void periodic() {
		// This method will be called once per scheduler run
		runStateMachine();
		updateDashboard();
	}

	public void runStateMachine() {
		switch(currentState){
			case INIT: {
				setState(TowerState.HOME);
				break;
			}
			
			case HOME: {
				if (joystick.getLeftBumperButton()){
					wrist.setIntakeSpeed(Constants.WristConstants.kCoralIntakePower);
					setState(TowerState.INTAKING);
				}
				break;
			}

	
		}
	}

	private void updateDashboard() {}

	private void setState(TowerState newState){
		currentState = newState;
		stateTimer.reset();
	}
}
