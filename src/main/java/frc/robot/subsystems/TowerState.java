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
	PAUSING_AFTER_SCORING_CORAL,

	//Algae
	FLIPING_WRIST_TO_SAFE,
	GOING_TO_ALGAE_L3,
	WAITING_FOR_ALGAE,
	CHANGING_ALGAE_HEIGHT,
	PAUSING_AFTER_SCORING_ALGAE,

	//Returning
	LOWERING,
	GOING_TO_INTAKE
}
