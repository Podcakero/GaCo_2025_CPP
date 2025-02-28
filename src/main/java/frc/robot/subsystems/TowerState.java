package frc.robot.subsystems;

public enum TowerState {
	//Start
	INIT,
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

	//Returning
	RETURNING_TO_SAFE, // L4
	LOWERING,
	GOING_TO_INTAKE
}
