
package roadgraph;

import geography.GeographicPoint;

import java.util.Collection;
import java.util.HashMap;
import java.util.HashSet;
import java.util.LinkedList;
import java.util.List;
import java.util.PriorityQueue;
import java.util.Queue;
import java.util.Set;
import java.util.function.Consumer;

import util.GraphLoader;

/**
 * @author UCSD MOOC development team and YOU
 *
 *         A class which represents a graph of geographic locations Nodes in the graph are intersections of multiple
 *         roads. Edges are the roads.
 *
 */
public class MapGraph {

	 
	private HashMap<GeographicPoint, mapVertice> verticeLoc;
	private HashSet<mapEdge> edges;
	
	public MapGraph() 
	{
		verticeLoc = new HashMap<GeographicPoint, mapVertice>();
		edges = new HashSet<mapEdge>();
	}

	/**
	 * Get the number of vertices (road intersections) in the graph
	 * 
	 * @return The number of vertices in the graph.
	 */
	public int getNumVertices() 
	{
		return verticeLoc.values().size();
	}

	/**
	 * Get the number of road segments in the graph
	 * 
	 * @return The number of edges in the graph.
	 */
	public int getNumEdges() 
	{
		return edges.size();
	}

	/**
	 * Add a node corresponding to an intersection at a Geographic Point
	 *
	 * @param location
	 *            The location of the intersection
	 */

	public boolean addVertex(GeographicPoint location) 
	{
		mapVertice n = verticeLoc.get(location);
		if (n == null) 
		{
			n = new mapVertice(location);
			verticeLoc.put(location, n);
			return true;
		} 
		return false;
	}



	public void addEdge(GeographicPoint fromPt, GeographicPoint toPt, String roadName, String roadType, double length) 
	{
		mapVertice from= verticeLoc.get(fromPt);
		mapVertice to= verticeLoc.get(toPt);

		// check nodes are valid 
		if (from == null)
			throw new NullPointerException("addEdge: pt1:" +from+ "is not in graph");
		if (to == null)
			throw new NullPointerException("addEdge: pt2:" +to+ "is not in graph");

		addEdge(from, to, roadName, roadType, length);
	}
 
	public boolean isNode(GeographicPoint point) 
	{
		return verticeLoc.containsKey(point);
	}


	private void addEdge(mapVertice n1, mapVertice n2, String roadName, String roadType, double length) 
	{
		mapEdge edge = new mapEdge(roadName, roadType, n1, n2, length);
		edges.add(edge);
		n1.addEdge(edge);
	}
 
	public Set<GeographicPoint> getVertices() {
		return verticeLoc.keySet();
	}
 
	private Set<mapVertice> getNeighbors(mapVertice node) {
		return node.getNeighbors();
	}

	public List<GeographicPoint> bfs(GeographicPoint start, GeographicPoint goal) 
	{
		Consumer<GeographicPoint> temp = (x) -> {};
		return bfs(start, goal, temp);
	}

	/**
	 * Find the path from start to goal using Breadth First Search
	 *
	 * @param start
	 *            The starting location
	 * @param goal
	 *            The goal location
	 * @return The list of intersections that form the shortest path from start to goal (including both start and goal).
	 */
	public List<GeographicPoint> bfs(GeographicPoint start, GeographicPoint goal, Consumer<GeographicPoint> nodeSearched) 
	{
		 
		if (start == null || goal == null || verticeLoc.get(start)==null || verticeLoc.get(goal)==null)
		throw new NullPointerException("Cannot find route from or to null node");
		
		mapVertice startNode = verticeLoc.get(start);
		mapVertice endNode = verticeLoc.get(goal);
	 
		HashMap<mapVertice, mapVertice> parentMap = new HashMap<mapVertice, mapVertice>();
		Queue<mapVertice> toSearch = new LinkedList<mapVertice>();
		HashSet<mapVertice> visited = new HashSet<mapVertice>();
		toSearch.add(startNode);
		mapVertice curr = null;

		while (!toSearch.isEmpty()) 
		{
			curr = toSearch.remove();

			// hook for visualization
			nodeSearched.accept(curr.getLocation());

			if (curr.equals(endNode))
				break;

			for (mapVertice neighbor : getNeighbors(curr)) 
			{
				if (!visited.contains(neighbor)) 
				{
					visited.add(neighbor);
					parentMap.put(neighbor, curr);
					toSearch.add(neighbor);
				}
			}
		}
		if (!curr.equals(endNode)) 
		{
			System.out.println("No path found from " + start + " to " + goal);
			return null;
		}
 
		return reconstructPath(parentMap, startNode, endNode);
	}

	/**
	 * Reconstruct a path from start to goal using the parentMap
	 *
	 * @param parentMap
	 *            the HashNode map of children and their parents
	 * @param start
	 *            The starting location
	 * @param goal
	 *            The goal location
	 * @return The list of intersections that form the shortest path from start to goal (including both start and goal).
	 */

	private List<GeographicPoint> reconstructPath(HashMap<mapVertice, mapVertice> parentMap, mapVertice start, mapVertice goal) 
	{
		
		LinkedList<GeographicPoint> path = new LinkedList<GeographicPoint>();
		mapVertice current = goal;

		while (!current.equals(start)) 
		{
			path.addFirst(current.getLocation());
			current = parentMap.get(current);
		}

		path.addFirst(start.getLocation());
		return path;
	}
	

	/**
	 * Find the path from start to goal using Dijkstra's algorithm
	 * 
	 * @param start
	 *            The starting location
	 * @param goal
	 *            The goal location
	 * @return The list of intersections that form the shortest path from start to goal (including both start and goal).
	 */

//
//	public List<GeographicPoint> dijkstra(GeographicPoint start, GeographicPoint goal) {
//		// Dummy variable for calling the search algorithms
//		// You do not need to change this method.
//        Consumer<GeographicPoint> temp = (x) -> {};
//        //return dijkstra(start, goal, temp); 
//        return aOrD(start,goal,temp,false);
//	}
//	
	
	public List<GeographicPoint> dijkstra(GeographicPoint start, GeographicPoint goal, Consumer<GeographicPoint> nodeSearched) 
	{

		if (start == null || goal == null || verticeLoc.get(start)==null || verticeLoc.get(goal)==null)
			throw new NullPointerException("Cannot find route from or to null node");

		
		mapVertice startNode = verticeLoc.get(start);
		mapVertice endNode = verticeLoc.get(goal);

		 

		HashMap<mapVertice, mapVertice> parentMap = new HashMap<mapVertice, mapVertice>();
 		PriorityQueue<mapVertice> toSearch = new PriorityQueue<mapVertice>();
		HashSet<mapVertice> visited = new HashSet<mapVertice>();
		mapVertice curr = null;

		int count = 0;

		for (GeographicPoint pt : getVertices()) 
		{
			verticeLoc.get(pt).setDistance(Double.POSITIVE_INFINITY);
			verticeLoc.get(pt).setAimDistance(Double.POSITIVE_INFINITY);
		}

		startNode.setDistance(0.0);
		startNode.setAimDistance(0.0);
		toSearch.add(startNode);

		while (!toSearch.isEmpty()) {
			curr = toSearch.remove();
			nodeSearched.accept(curr.getLocation());
			if (!visited.contains(curr)) 
			{
				visited.add(curr);
				count++;
				
				if (curr.equals(endNode)) break;

				for (mapVertice neighbor :  getNeighbors(curr)) 
				{
					if (!visited.contains(neighbor)) 
					{
						Double distFCurr = distNeighbor(curr.getLocation(), neighbor.getLocation());
						if (distFCurr + curr.getDistance() < neighbor.getDistance()) 
						{
							neighbor.setAimDistance(distFCurr + curr.getAimDistance());
							neighbor.setDistance(neighbor.getAimDistance());
							parentMap.put(neighbor, curr);
							toSearch.add(neighbor);
						}
					}
				}

			}
		}

		System.out.println("No of nodes visited: " + count);

		boolean flag= curr.equals(endNode);
		if (!flag)
		{
			System.out.println("No path found from " + start + " to " + goal);
			return null;
		}

	return reconstructPath(parentMap, startNode, endNode);
	}

	
	/**
	 * Find the path from start to goal using Dijkstra's algorithm
	 * 
	 * @param start
	 *            The starting location
	 * @param goal
	 *            The goal location
	 * @param nodeSearched
	 *            A hook for visualization. See assignment instructions for how to use it.
	 * @return The list of intersections that form the shortest path from start to goal (including both start and goal).
	 */
	

	/**
	 * Find the path from start to goal using A-Star search
	 * 
	 * @param start
	 *            The starting location
	 * @param goal
	 *            The goal location
	 * @return The list of intersections that form the shortest path from start to goal (including both start and goal).
	 */

	public List<GeographicPoint> dijkstra(GeographicPoint start, GeographicPoint goal) {
		// Dummy variable for calling the search algorithms
		// You do not need to change this method.
        Consumer<GeographicPoint> temp = (x) -> {};
        //return dijkstra(start, goal, temp); 
        return aOrD(start,goal,temp,false);
	}
	
	
	public List<GeographicPoint> aStarSearch(GeographicPoint start, GeographicPoint goal) 
	{
		// Dummy variable for calling the search algorithms
		// You do not need to change this method.
		Consumer<GeographicPoint> temp = (x) -> {};
		return aOrD(start, goal, temp,true);
	}

	public List<GeographicPoint> aOrD(GeographicPoint start, GeographicPoint goal, Consumer<GeographicPoint> nodeSearched,boolean aStar) 
	{
		
		if (start == null || goal == null || verticeLoc.get(start)==null || verticeLoc.get(goal)==null)
			throw new NullPointerException("Cannot find route from or to null node");

		mapVertice startNode = verticeLoc.get(start);
		mapVertice endNode = verticeLoc.get(goal);
 
		HashMap<mapVertice, mapVertice> parentMap = new HashMap<mapVertice, mapVertice>();
 		PriorityQueue<mapVertice> toSearch = new PriorityQueue<mapVertice>();
		HashSet<mapVertice> visited = new HashSet<mapVertice>();
		mapVertice curr = null;
		int count = 0;

		for (GeographicPoint pt : getVertices()) 
		{
			verticeLoc.get(pt).setDistance(Double.POSITIVE_INFINITY);
			verticeLoc.get(pt).setAimDistance(Double.POSITIVE_INFINITY);
		}


		startNode.setDistance(0.0);
		startNode.setAimDistance(0.0);
		toSearch.add(startNode);


		while (!toSearch.isEmpty()) 
		{
			curr = toSearch.remove();
			nodeSearched.accept(curr.getLocation());

			if (!visited.contains(curr)) 
			{
				visited.add(curr);
				count++;
				if (curr.equals(endNode)) break; 
				
				for (mapVertice neighbor : getNeighbors(curr)) 
				{
					if (!visited.contains(neighbor)) 
					{
						Double distFCurr = distNeighbor(curr.getLocation(), neighbor.getLocation());
						Double distFTarg=0.0;
						if(aStar) 
						distFTarg= displacement(neighbor.getLocation(), endNode.getLocation());
						//diff between dijkstra and astart is to check diff from goal of current pt (aim distance)

						if (distFCurr + curr.getDistance() < neighbor.getDistance()) 
						{
							neighbor.setAimDistance(distFCurr + curr.getAimDistance());
							neighbor.setDistance(neighbor.getAimDistance() + distFTarg);
							parentMap.put(neighbor, curr);
							toSearch.add(neighbor);
						}
					}
				}

			}
		}

		System.out.println("No of visited nodes aord : " + count);

		boolean flag= curr.equals(endNode);
		if (!flag) {
			System.out.println("No path found from " + start + " to " + goal);
			return null;
		}

	return reconstructPath(parentMap, startNode, endNode);
	}



	/**
	 * Find the distance between 2 neighboring nodes using the edge that connects them
	 * 
	 * @param start
	 *            The starting location
	 * @param end
	 *            The ending location
	 * @return The double number representing the distance between the 2 selected nodes
	 */
	public Double distNeighbor(GeographicPoint start, GeographicPoint end) 
	{
		for (mapEdge e : this.edges) 
		{
			if (e.getEndNode().getLocation().equals(end) 
					&& e.getOtherNode(verticeLoc.get(end)).getLocation().equals(start)) 
				return e.getLength();
			
			if (start.equals(end)) return 0.0;
			
		}
		return null;
	}

	/**
	 * Find the straight line between 2 nodes using their geometric data
	 * 
	 * @param start
	 *            The starting location
	 * @param end
	 *            The ending location
	 * @return The double number representing the straight line between the 2 selected nodes
	 */
	public Double displacement(GeographicPoint start, GeographicPoint end) 
	{
		// The actual geometric distance is multiplied by a factor of 100 to
		// resemble distance in kilometers
		return 100 * Math.sqrt(Math.pow(end.getX() - start.getX(), 2.0) + Math.pow(end.getY() - start.getY(), 2.0));
	}
	
	

	// main method for testing
	public static void main(String[] args) {
	
//		  System.out.print("Making a new map...");
//		 
//		MapGraph firstMap = new MapGraph();
//		System.out.print("DONE. \nLoading the map...");
//		GraphLoader.loadRoadMap("data/testdata/simpletest.map", firstMap);
//		System.out.println("DONE.");
//		GeographicPoint testStart = new GeographicPoint(1.0, 1.0);
//		GeographicPoint testEnd = new GeographicPoint(8.0, -1.0);
//		
//		System.out.println("Test 1 using simpletest: BFS ");
//		List<GeographicPoint> testroute = firstMap.bfs(testStart,testEnd);
//		System.out.println(testroute);
//		
		
		
//		MapGraph simpleTestMap = new MapGraph();
//		GraphLoader.loadRoadMap("data/testdata/simpletest.map", simpleTestMap);
//		
//		GeographicPoint testStart = new GeographicPoint(1.0, 1.0);
//		GeographicPoint testEnd = new GeographicPoint(8.0, -1.0);
//		
//		System.out.println("Test 1 using simpletest: Dijkstra should be 9 and AStar should be 5");
//		List<GeographicPoint> testroute = simpleTestMap.dijkstra(testStart,testEnd);
//		List<GeographicPoint> testroute2 = simpleTestMap.aStarSearch(testStart,testEnd);
//		
//		System.out.println(testroute);
//		System.out.println(testroute2);
//		
//		double dist=0.0;
//		for(int i=0;i<testroute.size();i++)
//		{
//			GeographicPoint p= testroute.get(i);
//			GeographicPoint n= testroute.get(i+1);
//			if(n.distance(testEnd)==0) {System.out.println(dist); break;}
//			else dist+=p.distance(n);
//		}
		MapGraph simpleTestMap = new MapGraph();
 		GraphLoader.loadRoadMap("data/testdata/simpletest.map", simpleTestMap);
 		
		GeographicPoint testStart = new GeographicPoint(1.0, 1.0);
		GeographicPoint testEnd = new GeographicPoint(8.0, -1.0);
		System.out.println("Test 1 using simpletest: Dijkstra should be 9 and AStar should be 5");
		List<GeographicPoint> testroute = simpleTestMap.dijkstra(testStart,testEnd);
		List<GeographicPoint> testroute2 = simpleTestMap.aStarSearch(testStart,testEnd);
		
		
		MapGraph testMap = new MapGraph();
		GraphLoader.loadRoadMap("data/maps/utc.map", testMap);
		
		// A very simple test using real data
		testStart = new GeographicPoint(32.869423, -117.220917);
		testEnd = new GeographicPoint(32.869255, -117.216927);
		System.out.println("Test 2 using utc: Dijkstra should be 13 and AStar should be 5");
		testroute = testMap.dijkstra(testStart,testEnd);
		testroute2 = testMap.aStarSearch(testStart,testEnd);
		
		
		// A slightly more complex test using real data
		testStart = new GeographicPoint(32.8674388, -117.2190213);
		testEnd = new GeographicPoint(32.8697828, -117.2244506);
		System.out.println("Test 3 using utc: Dijkstra should be 37 and AStar should be 10");
		testroute = testMap.dijkstra(testStart,testEnd);
		testroute2 = testMap.aStarSearch(testStart,testEnd);
		 
		
		
		/* Use this code in Week 3 End of Week Quiz */
		/*MapGraph theMap = new MapGraph();
		System.out.print("DONE. \nLoading the map...");
		GraphLoader.loadRoadMap("data/maps/utc.map", theMap);
		System.out.println("DONE.");

		GeographicPoint start = new GeographicPoint(32.8648772, -117.2254046);
		GeographicPoint end = new GeographicPoint(32.8660691, -117.217393);
		
		
		List<GeographicPoint> route = theMap.dijkstra(start,end);
		List<GeographicPoint> route2 = theMap.aStarSearch(start,end);

//		*/

		
		/* Use this code in Week 3 End of Week Quiz */
		MapGraph theMap = new MapGraph();
		System.out.print("DONE. \nLoading the map...");
		GraphLoader.loadRoadMap("data/maps/utc.map", theMap);
		System.out.println("DONE.");

		GeographicPoint start = new GeographicPoint(32.8648772, -117.2254046);
		GeographicPoint end = new GeographicPoint(32.8660691, -117.217393);
		
		
		List<GeographicPoint> route = theMap.dijkstra(start,end);
		List<GeographicPoint> route2 = theMap.aStarSearch(start,end);
		 

		
	}
	

}