var WINDOW_X = 800;
var WINDOW_Y = 500;
var AXES_OFFSET = 50;

var robot = [ [ 400, 30 ], [ 420, 70 ], [ 440, 50 ] ];

var start = [ 400, 30 ];
var goal = [ 50, 260 ];

var obstacles = [ [ [ 310, 110 ], [ 330, 130 ], [ 350, 110 ] ],
		[ [ 100, 200 ], [ 130, 130 ], [ 150, 110 ], [ 200, 150 ] ] ];

//Set up for drawing lines using p5 library
function setup() {

	createCanvas(WINDOW_X, WINDOW_Y);
	stroke('black');
	line(AXES_OFFSET, 0, AXES_OFFSET, WINDOW_Y);
	line(0, WINDOW_Y - AXES_OFFSET, WINDOW_X, WINDOW_Y - AXES_OFFSET);

	noFill();
	stroke('red');
	drawObject(robot);

	stroke('blue');
	for (var i = 0; i < obstacles.length; i++) {
		var obstacle = obstacles[i];
		drawObject(obstacle);
	}

	stroke('purple');
	drawCircle(goal, 5);

	// set a reference point on robot. Here 1st point in the array.
	var robotOrigin = robot[0];
	fill('yellow');

	noFill();
	drawCircle(robotOrigin, 5);

	// For finding the reflected robot vertices
	var reflectedRobot = [];
	for (var k = 0; k < robot.length; k++) {
		//For each vertex of original robot
		var robotVtx = robot[k];
		//coordinates for each vertex of reflected robot
		var reflectedX = (robotOrigin[0] - robotVtx[0]);
		var reflectedY = (robotOrigin[1] - robotVtx[1]);
		//add the new found vertex to the reflected robot vertices list
		reflectedRobot.push([ reflectedX, reflectedY ]);
	}
	stroke('pink');
	drawObject(reflectedRobot);
	// For finding C-obstacle vertices for each obstacle
	var cObstacles = [];
	//For each obstacle 
	for (var i = 0; i < obstacles.length; i++) {
		var obstacle = obstacles[i];
		// Minkowski points list
		var minkPoints = [];
		//for each vertex of obstacle
		for (var j = 0; j < obstacle.length; j++) {
			var obstacleVtx = obstacle[j];
			//for each reflected robot vertex
			for (var k = 0; k < reflectedRobot.length; k++) {
				var robotVtx = reflectedRobot[k];
				// Minkowski sum of obstacle vertex and Reflected robot vertex
				var minkPointX = obstacleVtx[0] + robotVtx[0];
				var minkPointY = obstacleVtx[1] + robotVtx[1];
				//Minkowski point of C- obstacle added to minkowski points list
				minkPoints.push([ minkPointX, minkPointY ]);
			}
		}
		//Finding convex hull with the minkowski points and save it as a c-obstacle of ith obstacle
		var minkHull = convexHull(minkPoints);
		// Add minkHull of ith obstacle to c -Obstacle list
		cObstacles.push(minkHull);
	}

	stroke('green');
	for (var i = 0; i < cObstacles.length; i++) {
		var cObstacle = cObstacles[i];
		drawObject(cObstacle);
	}

	var nodes = {};
	nodes["start"] = start;
	nodes["goal"] = goal;
	//For each cObstacle
	for (var i = 0; i < cObstacles.length; i++) {
		var currentObstacle = cObstacles[i];
		//for each cObstacle vertex
		for (var j = 0; j < currentObstacle.length; j++) {
			var currentVtx = currentObstacle[j];
			var currentVtxName = vertexName(i, j);
			nodes[currentVtxName] = currentVtx;
		}
	}

	var adjacencyMatrix = {};
	for ( var i in nodes) {
		adjacencyMatrix[i] = {};
		for ( var j in nodes) {
			adjacencyMatrix[i][j] = false;
		}
	}

	var obstacleEdges = [];
	for (var i = 0; i < cObstacles.length; i++) {
		var currentObstacle = cObstacles[i];
		for (var j = 0; j < currentObstacle.length; j++) {
			var currentVtx = currentObstacle[j];
			var currentVtxName = vertexName(i, j);
			var nextVtx = null;
			var nextVtxName = null;
			if (j == currentObstacle.length - 1) {
				nextVtx = currentObstacle[0];
				nextVtxName = vertexName(i, 0);
				obstacleEdges.push([ currentVtx, nextVtx ]);
			} else {
				nextVtx = currentObstacle[j + 1];
				nextVtxName = vertexName(i, j + 1);
			}
			obstacleEdges.push([ currentVtx, nextVtx ]);
			adjacencyMatrix[currentVtxName][nextVtxName] = true;
			adjacencyMatrix[nextVtxName][currentVtxName] = true;
		}
	}

	var visibilityEdge = [ start, goal ];
	if (!doesCollideWithObstacles(visibilityEdge, obstacleEdges)) {
		adjacencyMatrix["start"]["goal"] = true;
	}

	for (var i = 0; i < cObstacles.length; i++) {
		var currentObstacle = cObstacles[i];
		for (var j = 0; j < currentObstacle.length; j++) {
			var currentVtxName = vertexName(i, j);
			var currentVtx = currentObstacle[j];
			var visibilityEdge = [ start, currentVtx ];
			if (!doesCollideWithObstacles(visibilityEdge, obstacleEdges)) {
				adjacencyMatrix["start"][currentVtxName] = true;
				adjacencyMatrix[currentVtxName]["start"] = true;
			}
		}
	}

	for (var i = 0; i < cObstacles.length; i++) {
		var currentObstacle = cObstacles[i];
		for (var j = 0; j < currentObstacle.length; j++) {
			var currentVtxName = vertexName(i, j);
			var currentVtx = currentObstacle[j];
			var visibilityEdge = [ goal, currentVtx ];
			if (!doesCollideWithObstacles(visibilityEdge, obstacleEdges)) {
				adjacencyMatrix[currentVtxName]["goal"] = true;
			}
		}
	}

	for (var w = 0; w < cObstacles.length; w++) {
		var currentObstacle = cObstacles[w];
		for (var x = 0; x < currentObstacle.length; x++) {
			var currentObstacleVtx = currentObstacle[x];
			var currentObstacleVtxName = vertexName(w, x);
			for (var y = 0; y < cObstacles.length; y++) {
				var otherObstacle = cObstacles[y];
				if (w != y) {
					for (var z = 0; z < otherObstacle.length; z++) {
						var otherObstacleVtx = otherObstacle[z];
						var otherObstacleVtxName = vertexName(y, z);
						var visibilityEdge = [ currentObstacleVtx, otherObstacleVtx ];
						if (!doesCollideWithObstacles(visibilityEdge, obstacleEdges)) {
							adjacencyMatrix[currentObstacleVtxName][otherObstacleVtxName] = true;
							adjacencyMatrix[otherObstacleVtxName][otherObstacleVtxName] = true;
						}
					}
				}
			}
		}
	}

	stroke('darkgreen');
	for ( var i in adjacencyMatrix) {
		for ( var j in adjacencyMatrix[i]) {
			if (adjacencyMatrix[i][j])
				drawLine([ nodes[i], nodes[j] ]);
		}
	}

	stroke('red');
	drawLines(obstacleEdges);

	// Djikstra's algorithm
	var tentativeDistance = {};
	var isVisited = {};
	var path = {};
	for ( var i in nodes) {
		tentativeDistance[i] = Number.MAX_VALUE;
		isVisited[i] = false;
		path[i] = null;
	}

	var currentNodeIndex = "start";
	tentativeDistance["start"] = 0;
	path["start"] = [ "start" ];

	do {
		var currentNode = nodes[currentNodeIndex];
		var adjacencyColumn = adjacencyMatrix[currentNodeIndex];
		for ( var i in adjacencyColumn) {
			if (adjacencyColumn[i] == true) {
				if (isVisited[i] == false) {
					var edgeDistance = distanceBetween(nodes[i], currentNode);
					var newTentativeDistance = tentativeDistance[currentNodeIndex]
							+ edgeDistance;
					if (newTentativeDistance < tentativeDistance[i]) {
						tentativeDistance[i] = newTentativeDistance;
						path[i] = path[currentNodeIndex].concat([ i ]);
					}
				}
			}
		}
		isVisited[currentNodeIndex] = true;

		var smallestDistanceInUnVisited = Number.MAX_VALUE;
		var smallestDistanceNodeIndex = null;
		for ( var i in nodes) {
			if (isVisited[i] == false) {
				if (tentativeDistance[i] < smallestDistanceInUnVisited) {
					smallestDistanceInUnVisited = tentativeDistance[i];
					smallestDistanceNodeIndex = i;
				}
			}
		}

		if (isVisited["goal"] == true
				|| smallestDistanceInUnVisited == Number.MAX_VALUE) {
			break;
		}

		currentNodeIndex = smallestDistanceNodeIndex;
	} while (true);

	stroke('yellow');
	var shortestPath = path["goal"];
	for (var i = 0; i < shortestPath.length - 1; i++) {
		drawLine([ nodes[shortestPath[i]], nodes[shortestPath[i + 1]] ]);
	}

}

function distanceBetween(point1, point2) {
	return Math.sqrt(Math.pow(point2[0] - point1[0], 2)
			+ Math.pow(point2[1] - point1[1], 2));
}

function vertexName(obstacleIndex, vertexIndex) {
	return "o-" + obstacleIndex + "-" + vertexIndex;
}

function doesCollideWithObstacles(visibilityEdge, obstacleEdges) {

	var visibilityPoint0 = visibilityEdge[0];
	var visibilityPoint1 = visibilityEdge[1];

	var doesCollide = false;
	for (var j = 0; j < obstacleEdges.length; j++) {
		var obstacleEdge = obstacleEdges[j];
		var obstaclePoint0 = obstacleEdge[0];
		var obstaclePoint1 = obstacleEdge[1];

		doesCollide = collideLineLine(visibilityPoint0[0], visibilityPoint0[1],
				visibilityPoint1[0], visibilityPoint1[1], obstaclePoint0[0],
				obstaclePoint0[1], obstaclePoint1[0], obstaclePoint1[1], false);

		if (doesCollide == true) {
			break;
		}
	}
	return doesCollide;
}

function drawLines(lines) {
	for (var i = 0; i < lines.length; i++) {
		drawLine(lines[i]);
	}
}

function drawLine(ln) {
	var point1 = ln[0];
	var newPoint1 = changeCordinates(point1);

	var point2 = ln[1];
	var newPoint2 = changeCordinates(point2);

	line(newPoint1[0], newPoint1[1], newPoint2[0], newPoint2[1]);
}

function convexHull(points) {
	var hullIndexes = [];

	var leftMostIndex = leftMost(points);
	hullIndexes.push(leftMostIndex);

	var justInsertedIndex = leftMostIndex;

	while (true) {
		var candidateIndex = null;
		for (var i = 0; i < points.length; i++) {
			var currentIndex = i;
			var currentPoint = points[i];
			if (currentIndex == justInsertedIndex) {
				continue;
			} else {
				if (candidateIndex == null) {
					candidateIndex = currentIndex;
					continue;
				} else {
					var justInsertedPoint = points[justInsertedIndex];
					var candidatePoint = points[candidateIndex];
					if (isLeft(justInsertedPoint, candidatePoint, currentPoint)) {

						candidateIndex = currentIndex;
					}
				}
			}
		}
		if (candidateIndex == leftMostIndex) {
			// We came back to the first point
			break;
		} else {
			hullIndexes.push(candidateIndex);
			justInsertedIndex = candidateIndex;
		}
	}

	var hull = [];
	for (var i = 0; i < hullIndexes.length; i++) {
		var hullIndex = hullIndexes[i];
		var hullPoint = points[hullIndex];
		hull.push(hullPoint);
	}
	return hull;
}

function leftMost(points) {
	var firstPoint = points[0]; // (x,y)
	var minX = firstPoint[0]; // x
	var minIndex = 0;
	for (var i = 0; i < points.length; i++) {
		var point = points[i];
		var pointX = point[0];
		if (pointX < minX) {
			minX = pointX;
			minIndex = i;
		}
	}
	return minIndex;
}

function drawCircle(center, radius) {
	var newCenter = changeCordinates(center);
	ellipse(newCenter[0], newCenter[1], radius, radius);
}

function drawObject(vertices) {
	beginShape();
	for (var j = 0; j < vertices.length; j++) {
		var vtx = vertices[j];
		var newVtx = changeCordinates(vtx);
		vertex(newVtx[0], newVtx[1]);
	}
	endShape(CLOSE);
}

function changeCordinates(point) {
	var changedPoint = [];
	changedPoint[0] = point[0] + AXES_OFFSET;
	changedPoint[1] = WINDOW_Y - point[1] - AXES_OFFSET;
	return changedPoint;
}

function isLeft(p1, p2, p3) {
	return ((p3[0] - p1[0]) * (p2[1] - p1[1]) > (p3[1] - p1[1]) * (p2[0] - p1[0]));
}
