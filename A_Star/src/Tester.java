
public class Tester {
	public static void main(String[] args){
		AStarAlgorithm astar = new AStarAlgorithm();
		priorityQueue p = new priorityQueue();
		astar.initialize();
		p = reverse(astar.AStar(astar.getCell(2,0), astar.getCell(0,2)));
		for(int i = 0; i < p.size; i++){
			System.out.println(p.elements[i].c.x + "," + p.elements[i].c.y);
		}
		if(p.size == 0){
			System.out.println("LOL NO PATH!");
		}
		Movement movePath = new Movement();
		movePath = movementPath(p, 0.0);
		for(int l = 0; l < movePath.size; l++){
			System.out.println("Dist: " + movePath.moveP[l].dist + " Type: " + movePath.moveP[l].movementType.name());
		}
	}
	
	public static priorityQueue reverse(priorityQueue path){
		priorityQueue newP = new priorityQueue();
		for(int i = path.size - 1; i >= 0; i--){
			newP.elements[-(i - (path.size - 1))] = path.elements[i];
		}
		newP.size = path.size;
		return newP;
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
