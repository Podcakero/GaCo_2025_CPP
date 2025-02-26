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
	TILTING_TO_SCORE, // L4
	SCORING_CORAL,

	//Returning
	RETURNING_TO_SAFE, // L4
	LOWERING,
	GOING_TO_INTAKE
}
