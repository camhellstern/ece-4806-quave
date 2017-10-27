package org.firstinspires.ftc.teamcode;

/**
 * Created by Cameron on 10/26/2017.
 */

enum MessageType{
    OpDistance1, OpDistance2, OpDistance3, OpDistance4;
}

public class RoverMessage {
    MessageType msgType;
    double opDistVoltage = 0;
}
