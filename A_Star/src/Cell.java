
enum State {
    NEW, OPEN, CLOSED, LOWER, RAISED, NOTHING;
}
public class Cell {
    int x = 0;
    int y = 0;
    double g = 0;
    double f = 0;
    State state = State.NEW;
    int obstacle = 0;
}