package org.firstinspires.ftc.teamcode;

/**
 * Created by Cameron on 10/1/2017.
 */
enum State
{
    NEW, OPEN, CLOSED, LOWER, RAISED, NOTHING;
}
public class Cell
{
    int x = 0;
    int y = 0;
    double g = 0;
    double f = 0;
    State state = State.NEW;
    int obstacle = 0;
}


