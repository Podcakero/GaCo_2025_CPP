package frc.robot.subsystems;

public enum TowerState {
	//Start
	INIT,
	PREPARING_TO_HOME,
	LOWERING_TO_HOME,
	HOMING,
	HOME,

	//Scoring
	INTAKING,
	GOING_TO_SAFE,
	GOT_CORAL,
	RAISING,
	READY_TO_SCORE,
	RAISING_TO_L4,
	TILTING_TO_SCORE_L4, // L4
	SCORING_CORAL,
	PAUSING,

	//Algae
	FLIPING_WRIST_TO_SAFE,
	RAISING_LIFT_TO_ALGAE_HIGH,
	FLIPING_WRIST_TO_ALGAE_INTAKE,
	WAITING_FOR_ALGAE,
	SCORING_ALGAE,

	//Returning
	RETURNING_TO_SAFE, // L4
	LOWERING,
	GOING_TO_INTAKE
}
