package org.firstinspires.ftc.teamcode;


/**
 * Created by Cameron on 10/1/2017.
 */

public class AStarAlgorithm
{
    Node map[][] = new Node[27][19];
    int front = 0;
    int rear = 0;
    Node openList[] = new Node[513];
    priorityQueue closedList = new priorityQueue();
    //priorityQueue cf;
    priorityQueue path = new priorityQueue();

    public priorityQueue AStar(Node start, Node goal){
        for(int a = 0;a<513;a++)
        {
            openList[a] = new Node();
        }
        create();
        //initialize();
        setG(start, 0);
        start.c.f = calculateH(start, goal);
        insert_by_priority(start);
        //node *temp;
        Node current = new Node();
        while(front != -1)
        {
            current = openList[0];
            if(current.c.x == goal.c.x && current.c.y == goal.c.y)
            {
                calculatePath(current, start);
                return path;
            }
            delete_by_priority(current);
            current.c.state = State.CLOSED;
            placeInClosedList(current);
            Node neighbors[] = new Node[8];
            for(int b=0;b<8;b++)
            {
                neighbors[b] = new Node();
            }
            if((current.c.x+1) <= 26)
                neighbors[0] = map[current.c.x+1][current.c.y];
            else
            {
                Node temp2 = new Node();
                temp2.c.state = State.NOTHING;
                neighbors[0] = temp2;
            }
            if(((current.c.x+1) <= 26) && ((current.c.y+1) <= 18))
                neighbors[1] = map[current.c.x+1][current.c.y+1];
            else
            {
                Node temp2 = new Node();
                temp2.c.state = State.NOTHING;
                neighbors[1] = temp2;
            }
            if((current.c.y+1) <= 18)
                neighbors[2] = map[current.c.x][current.c.y+1];
            else
            {
                Node temp2 = new Node();
                temp2.c.state = State.NOTHING;
                neighbors[2] = temp2;
            }
            if(((current.c.x-1) >= 0) && ((current.c.y+1) <= 18))
                neighbors[3] = map[current.c.x-1][current.c.y+1];
            else
            {
                Node temp2 = new Node();
                temp2.c.state = State.NOTHING;
                neighbors[3] = temp2;
            }
            if((current.c.x-1) >= 0)
                neighbors[4] = map[current.c.x-1][current.c.y];
            else
            {
                Node temp2 = new Node();
                temp2.c.state = State.NOTHING;
                neighbors[4] = temp2;
            }
            if(((current.c.x-1) >= 0) && ((current.c.y-1) >= 0))
                neighbors[5] = map[current.c.x-1][current.c.y-1];
            else
            {
                Node temp2 = new Node();
                temp2.c.state = State.NOTHING;
                neighbors[5] = temp2;
            }
            if((current.c.y-1) >= 0)
                neighbors[6] = map[current.c.x][current.c.y-1];
            else
            {
                Node temp2 = new Node();
                temp2.c.state = State.NOTHING;
                neighbors[6] = temp2;
            }
            if(((current.c.x+1) <= 26) && ((current.c.y-1) >= 0))
                neighbors[7] = map[current.c.x+1][current.c.y-1];
            else
            {
                Node temp2 = new Node();
                temp2.c.state = State.NOTHING;
                neighbors[7] = temp2;
            }
            //cell temp2 = neighbors[1];
            for(int i=0;i<8;i++)
            {
                if(neighbors[i].c.state == State.NOTHING)
                {
                    continue;
                }
                analyzeNeighbor(neighbors[i], current, goal);
            }
        }

        return path;
    }

    private void create(){
        front = rear = -1;
    }

    public void initialize() {
        for (int x = 0; x < 27; x++) {
            for (int y = 0; y < 19; y++) {
                Node temp = new Node();
                temp.c.g = 100000;
                temp.c.f = 100000;
                temp.c.obstacle = 0;
                temp.c.x = x;
                temp.c.y = y;
                temp.c.state = State.NEW;
                map[x][y] = temp;
            }
        }
        //path.size = 0;
        closedList.size = 0;
    }

    private void setG(Node n, int g){
        n.c.g = g;
    }

    private double calculateH(Node start, Node goal){
        int x = goal.c.x - start.c.x;
        int y = goal.c.y - start.c.y;
        double h = Math.sqrt((x*x)+(y*y));
        return h;
    }
    private void insert_by_priority(Node data)
    {
        if (rear >= (20 - 1))
        {
            //printf("\nQueue overflow no more elements can be inserted");
            return;
        }
        if ((front == -1) && (rear == -1))
        {
            front++;
            rear++;
            openList[rear] = data;
            return;
        }
        else
            check(data);
        rear++;
    }

    private void check(Node data)
    {
        int i,j;

        for (i = 0; i <= rear; i++)
        {
            if (data.c.f < openList[i].c.f)
            {
                for (j = rear + 1; j > i; j--)
                {
                    openList[j] = openList[j - 1];
                }
                openList[i] = data;
                return;
            }
        }
        openList[i] = data;
    }

    private void calculatePath(Node current, Node start)
    {
        //path.size = 0;
        Node temp2 = current;
        path.elements[path.size] = temp2;
        path.size++;
        while((current.c.x != start.c.x) || (current.c.y != start.c.y))
        {
            for(int i=0;i<closedList.size;i++)
            {
                if(current.cameFrom.x == closedList.elements[i].c.x && current.cameFrom.y == closedList.elements[i].c.y)
                {
                    current = closedList.elements[i];
                    break;
                }
            }
            Node temp = current;
            path.elements[path.size] = temp;
            path.size++;
        }
        return;
    }

    private void delete_by_priority(Node data)
    {
        int i;

        if ((front==-1) && (rear==-1))
        {
            //printf("\nQueue is empty no elements to delete");
            return;
        }

        for (i = 0; i <= rear; i++)
        {
            if (data.c.x == openList[i].c.x && data.c.y == openList[i].c.y)
            {
                for (; i < rear; i++)
                {
                    openList[i] = openList[i + 1];
                }
                Node temp = new Node();
                openList[i] = temp;
                rear--;

                if (rear == -1)
                    front = -1;
                return;
            }
        }
        //printf("\n%d not found in queue to delete", data);
    }

    private void placeInClosedList(Node n)
    {
        closedList.elements[closedList.size] = n;
        closedList.size++;
    }

    private void analyzeNeighbor(Node n, Node source, Node goal)
    {
        boolean ignore = false;
        if(n.c.state == State.CLOSED)
        {
            return;
        }
        double tentative_g = source.c.g + distanceBetween(n, source);
        //double tentative_f = tentative_g + calculateH(*node, goal);if(node->s == NEW)
        if(n.c.state == State.NEW)
        {
            insert_by_priority(n);
            n.c.state = State.OPEN;
        }
        else if(tentative_g >= n.c.g)
        {
            return;
        }
        //path.elements[path.size] = node;
        //path.size++;
        n.cameFrom = source.c;
        n.c.g = tentative_g;
        n.c.f = n.c.g + calculateH(n, goal);
    /*for(int i=0;i<cf.size;i++)
    {
        if(n->c.x == cf.elements[i]->c.x && n->c.y == cf.elements[i]->c.y)
        {
            return;
        }
    }
    cf.elements[cf.size] = n;
    cf.size++;*/
    }

    private double distanceBetween(Node n, Node source)
    {
        if(n.c.obstacle == 1)
        {
            return 100000;
        }
        else
        {
            return Math.sqrt((Math.abs(n.c.x - source.c.x) + Math.abs(n.c.y - source.c.y)));
        }
    }

    public Node getCell(int x, int y)
    {
        return map[x][y];
    }

    public static Movement movementPath(priorityQueue path, double currDirection){
        int DISTANCE_PER_SQ = 24;
        Movement move = new Movement();
        int newPCounter = 0;

        //check first node slope and currDirection
        boolean isFirstUp = false;
        int firstSlope = -100;
        if(path.elements[1].c.x == path.elements[0].c.x)
            isFirstUp = true;
        else
            firstSlope = (path.elements[1].c.y - path.elements[0].c.y) /
                    (path.elements[1].c.x - path.elements[0].c.x);

        MovementNode firstTurn = new MovementNode();
        double firstTheta1 = 0;
        double firstTheta2 = 0;

        boolean firstTurnExists = false;
        if(isFirstUp && currDirection != 90.0){
            firstTheta1 = 90;
            firstTurnExists = true;
        }
        else if(firstSlope == 0 && currDirection != 0.0){
            firstTheta1 = 0;
            firstTurnExists = true;
        }
        else if(firstSlope == 1 && currDirection != 45.0){
            firstTheta1 = 45;
            firstTurnExists = true;
        }
        //is this the right comparison (225)?
        else if(firstSlope == -1 && currDirection != 225.0){
            firstTheta1 = 225;
            firstTurnExists = true;
        }

        if(firstTurnExists){
            firstTheta2 = currDirection;
            double delta = firstTheta2 - firstTheta1;
            if(delta < 0){
                firstTurn.movementType = MovementType.TURN_LEFT;
                firstTurn.dist = Math.abs(delta);
            }
            else{
                firstTurn.movementType = MovementType.TURN_RIGHT;
                firstTurn.dist = delta;
            }
            move.moveP[newPCounter] = firstTurn;
            newPCounter++;
        }

        for(int i = 0; i < path.size;){
            boolean isUp = false;
            int slope = -100;
            if(path.elements[i+1].c.x == path.elements[i].c.x)
                isUp = true;
            else
                slope = (path.elements[i+1].c.y - path.elements[i].c.y) /
                        (path.elements[i+1].c.x - path.elements[i].c.x);
            int dist = 1;
            int j;
            for(j = i + 1; j + 1 < path.size; j++){
                boolean isUp2 = false;
                int slope2 = -100;
                if(path.elements[j + 1].c.x == path.elements[j].c.x)
                    isUp2 = true;
                else
                    slope2 = (path.elements[j + 1].c.y - path.elements[j].c.y) /
                            (path.elements[j + 1].c.x - path.elements[j].c.x);
                if((isUp ^ isUp2) || (slope != slope2)){
                    MovementNode mNode = new MovementNode();
                    mNode.movementType = MovementType.FORWARD;
                    MovementNode straightNode = new MovementNode();
                    straightNode.movementType = MovementType.FORWARD;
                    straightNode.dist = DISTANCE_PER_SQ;
                    double theta1 = Math.toDegrees(Math.atan(slope));
                    double theta2 = Math.toDegrees(Math.atan(slope2));
                    if(isUp)
                        theta1 = 0;
                    if(isUp2)
                        theta2 = 0;
                    if(slope == 1 || slope == -1)
                        straightNode.dist = dist * DISTANCE_PER_SQ * Math.sqrt(2);
                    else
                        straightNode.dist = dist * DISTANCE_PER_SQ;

                    double deltaTheta = theta2 - theta1;
                    if(deltaTheta < 0){
                        mNode.dist = Math.abs(deltaTheta);
                        mNode.movementType = MovementType.TURN_LEFT;
                    }
                    else{
                        mNode.dist = deltaTheta;
                        mNode.movementType = MovementType.TURN_RIGHT;

                    }
                    move.moveP[newPCounter] = straightNode;
                    move.moveP[newPCounter + 1] = mNode;
                    newPCounter+=2;
                    i = j;
                    break;
                }
                else
                    dist++;
            }
            //last node
            if(j == path.size - 1){
                int currSlope = -100;
                //slope of second to last node and last node
                if(path.elements[i + 1].c.x != path.elements[i].c.x)
                    currSlope = (path.elements[j].c.y - path.elements[j - 1].c.y) /
                            (path.elements[j].c.x - path.elements[j - 1].c.x);

                MovementNode lastNode = new MovementNode();
                lastNode.movementType = MovementType.FORWARD;
                if(currSlope == 1 || currSlope == -1)
                    lastNode.dist = dist * DISTANCE_PER_SQ * Math.sqrt(2);
                else
                    lastNode.dist = dist * DISTANCE_PER_SQ;

                move.moveP[newPCounter] = lastNode;
                move.size = newPCounter + 1;
                return move;
            }
        }
        return move;
    }
}
