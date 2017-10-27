package org.firstinspires.ftc.teamcode;

/**
 * Created by Cameron on 10/17/2017.
 */

enum MovementType {
    FORWARD, TURN_LEFT, TURN_RIGHT;
}

public class MovementNode {
    MovementType movementType = MovementType.FORWARD;
    double dist = 0;
}
