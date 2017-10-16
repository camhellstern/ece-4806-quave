enum MovementType {
    FORWARD, TURN_LEFT, TURN_RIGHT;
}

public class MovementNode {
	MovementType movementType = MovementType.FORWARD;
	double dist = 0;
}
