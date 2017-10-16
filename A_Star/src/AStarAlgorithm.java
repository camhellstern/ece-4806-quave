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
        //Node end = getCell(goal.c.x, goal.c.y);
        //calculatePath(end, start);
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
}
