/**
 * @author UCSD MOOC development team and YOU
 * 
 * A class which reprsents a graph of geographic locations
 * Nodes in the graph are intersections between 
 *
 */
package roadgraph;


import java.util.ArrayList;
import java.util.HashMap;
import java.util.HashSet;
import java.util.LinkedList;
import java.util.List;
import java.util.Map;
import java.util.PriorityQueue;
import java.util.Queue;
import java.util.Set;
import java.util.function.Consumer;

import geography.GeographicPoint;
import util.GraphLoader;

/**
 * @author UCSD MOOC development team and YOU
 * 
 * A class which represents a graph of geographic locations
 * Nodes in the graph are intersections between 
 *
 */
public class MapGraph {
    
    // adjacency list
	private Map<GeographicPoint, List<DirectedEdge>> adjList;
	
	// number of edges in the graph
	private int numOfEdges;
	
	
	/** 
	 * Create a new empty MapGraph 
	 */
	public MapGraph()
	{
		adjList = new HashMap<GeographicPoint, List<DirectedEdge>>();
		numOfEdges = 0;
	}
	
	/**
	 * Get the number of vertices (road intersections) in the graph
	 * @return The number of vertices in the graph.
	 */
	public int getNumVertices()
	{
		return adjList.size();
	}
	
	/**
	 * Return the intersections, which are the vertices in this graph.
	 * @return The vertices in this graph as GeographicPoints
	 */
	public Set<GeographicPoint> getVertices()
	{
		return adjList.keySet();
	}
	
	/**
	 * Get the number of road segments in the graph
	 * @return The number of edges in the graph.
	 */
	public int getNumEdges()
	{
		return numOfEdges;
	}

	
	/** Add a node corresponding to an intersection at a Geographic Point
	 * If the location is already in the graph or null, this method does 
	 * not change the graph.
	 * @param location  The location of the intersection
	 * @return true if a node was added, false if it was not (the node
	 * was already in the graph, or the parameter is null).
	 */
	public boolean addVertex(GeographicPoint location)
	{
		if (adjList.containsKey(location)) {
		    // this vertex is already in the graph
		    return false;
		}
		
		// initialize the list to null
		adjList.put(location, null);
		
		return true;
	}
	
	/**
	 * Adds a directed edge to the graph from pt1 to pt2.  
	 * Precondition: Both GeographicPoints have already been added to the graph
	 * @param from The starting point of the edge
	 * @param to The ending point of the edge
	 * @param roadName The name of the road
	 * @param roadType The type of the road
	 * @param length The length of the road, in km
	 * @throws IllegalArgumentException If the points have not already been
	 *   added as nodes to the graph, if any of the arguments is null,
	 *   or if the length is less than 0.
	 */
	public void addEdge(GeographicPoint from, GeographicPoint to, String roadName,
			String roadType, double length) throws IllegalArgumentException {

		if (Double.compare(length, 0.0) < 0) {
		    throw new IllegalArgumentException("Length cannot be less than zero");
		}
		
		if (to == null || from == null || roadName == null || roadType == null ) {
		    throw new IllegalArgumentException("Argument cannot be null");
		}
		
		if (!adjList.containsKey(to) || !adjList.containsKey(from)) {
		    throw new IllegalArgumentException("Either 'from' or 'to' vertex was not found in the graph");
		}

		// add the 'to' vertex to the adjacency list of the 'from' vertex
		List<DirectedEdge> neighborList = adjList.get(from);
		if (neighborList == null) {
		    // lazy init the list
		    neighborList = new ArrayList<DirectedEdge>();
		    adjList.put(from, neighborList);
		}
		neighborList.add(new DirectedEdge(from, to, roadName, roadType, length));
		++numOfEdges;
	}
	

	/** Find the path from start to goal using breadth first search
	 * 
	 * @param start The starting location
	 * @param goal The goal location
	 * @return The list of intersections that form the shortest (unweighted)
	 *   path from start to goal (including both start and goal).
	 */
	public List<GeographicPoint> bfs(GeographicPoint start, GeographicPoint goal) {
		// Dummy variable for calling the search algorithms
        Consumer<GeographicPoint> temp = (x) -> {};
        return bfs(start, goal, temp);
	}
	
	/** Find the path from start to goal using breadth first search
	 * 
	 * @param start The starting location
	 * @param goal The goal location
	 * @param nodeSearched A hook for visualization.  See assignment instructions for how to use it.
	 * @return The list of intersections that form the shortest (unweighted)
	 *   path from start to goal (including both start and goal).
	 */
	public List<GeographicPoint> bfs(GeographicPoint start, 
			 					     GeographicPoint goal, Consumer<GeographicPoint> nodeSearched)
	{
	    if (!adjList.containsKey(start) || !adjList.containsKey(goal)) {
	        // either the start or the goal vertex was not found in the graph
	        return null;
	    }
	    
		Queue<GeographicPoint> queue = new LinkedList<GeographicPoint>();
		Set<GeographicPoint> visitedSet = new HashSet<GeographicPoint>();
		Map<GeographicPoint, GeographicPoint> parentMap = new HashMap<GeographicPoint, GeographicPoint>();
		
		queue.add(start);
		parentMap.put(start, null); // useful while constructing the path
		visitedSet.add(start);
		nodeSearched.accept(start); // pass the start vertex to GUI
		while(!queue.isEmpty()) {
		    GeographicPoint curr = queue.remove();
		    
		    nodeSearched.accept(curr); // pass the neighbor vertex to GUI
		    
		    if (curr.equals(goal)) {
		        // found the goal vertex
		        break;
		    }
		    List<DirectedEdge> neighborList = adjList.get(curr);
            if (neighborList != null) {
                for (DirectedEdge edge : neighborList) {
                    
                    GeographicPoint neighbor = edge.getEnd();
                    
                    // check if the vertex has been visited
                    if (!visitedSet.contains(neighbor)) {
                        visitedSet.add(neighbor); // mark as visited
                        parentMap.put(neighbor, curr); // set the parent vertex to the current vertex
                        queue.add(neighbor); // add the child to the queue
                    }
                }
            }
		}
		
		return constructPath(start, goal, parentMap);
	}
	
	/**
	 *  Method to construct the path out of the vertex to parent vertex mapping
	 * @param parentMap
	 * @return
	 */
	private List<GeographicPoint> constructPath(GeographicPoint start, GeographicPoint goal, Map<GeographicPoint, GeographicPoint> parentMap) {
	    LinkedList<GeographicPoint> path = null;
	    
	    // the goal vertex should be reachable from start
	    if (parentMap.containsKey(goal)) {
	        
	        path = new LinkedList<GeographicPoint>();
	        
	        // start with the goal vertex and keep adding parents to the beginning of the path list until start vertex
	        path.addFirst(goal);
	        GeographicPoint curr = goal;
	        do {
	            GeographicPoint parentVertex = parentMap.get(curr);
	            path.addFirst(parentVertex);
	            curr = parentVertex;
	        } while (!curr.equals(start)); // stop when we reach the start vertex
	    }
	    
	    return path;
	}
	

	/** Find the path from start to goal using Dijkstra's algorithm
	 * 
	 * @param start The starting location
	 * @param goal The goal location
	 * @return The list of intersections that form the shortest path from 
	 *   start to goal (including both start and goal).
	 */
	public List<GeographicPoint> dijkstra(GeographicPoint start, GeographicPoint goal) {
		// Dummy variable for calling the search algorithms
		// You do not need to change this method.
        Consumer<GeographicPoint> temp = (x) -> {};
        return dijkstra(start, goal, temp);
	}
	
	/** Find the path from start to goal using Dijkstra's algorithm
	 * 
	 * @param start The starting location
	 * @param goal The goal location
	 * @param nodeSearched A hook for visualization.  See assignment instructions for how to use it.
	 * @return The list of intersections that form the shortest path from 
	 *   start to goal (including both start and goal).
	 */
	public List<GeographicPoint> dijkstra(GeographicPoint start, 
										  GeographicPoint goal, Consumer<GeographicPoint> nodeSearched)
	{
	    return searchUtility(start, goal, nodeSearched, Algo.DIJKSTRA);
	}

	/** Find the path from start to goal using A-Star search
	 * 
	 * @param start The starting location
	 * @param goal The goal location
	 * @return The list of intersections that form the shortest path from 
	 *   start to goal (including both start and goal).
	 */
	public List<GeographicPoint> aStarSearch(GeographicPoint start, GeographicPoint goal) {
		// Dummy variable for calling the search algorithms
        Consumer<GeographicPoint> temp = (x) -> {};
        return aStarSearch(start, goal, temp);
	}
	
	/** Find the path from start to goal using A-Star search
	 * 
	 * @param start The starting location
	 * @param goal The goal location
	 * @param nodeSearched A hook for visualization.  See assignment instructions for how to use it.
	 * @return The list of intersections that form the shortest path from 
	 *   start to goal (including both start and goal).
	 */
	public List<GeographicPoint> aStarSearch(GeographicPoint start, 
											 GeographicPoint goal, Consumer<GeographicPoint> nodeSearched)
	{
				
		return searchUtility(start, goal, nodeSearched, Algo.ASTAR);
	}

	/** Find the path from start to goal using Dijkstra or A-Star search.
     * 
     * @param start The starting location
     * @param goal The goal location
     * @param nodeSearched A hook for visualization.
     * @param type Dijkstra or A-Star
     * @return The list of intersections that form the shortest path from 
     *   start to goal (including both start and goal).
     */
    private List<GeographicPoint> searchUtility(GeographicPoint start, GeographicPoint goal, Consumer<GeographicPoint> nodeSearched, Algo type)
    {
        /*int count = 0;*/
        
        if (!adjList.containsKey(start) || !adjList.containsKey(goal)) {
            // either the start or the goal vertex was not found in the graph
            return null;
        }
        
        // init
        PriorityQueue<Distance> priorityQueue = new PriorityQueue<Distance>();
        Map<GeographicPoint, Distance> distances = initDistances(start, goal, type);
        Map<GeographicPoint, GeographicPoint> parentMap = new HashMap<GeographicPoint, GeographicPoint>();
        parentMap.put(start, null); // useful while constructing the path
        
        // enqueue start vertex
        priorityQueue.add(distances.get(start));
        
        while(!priorityQueue.isEmpty()) {
            // dequeue vertex with highest priority
            Distance currDistance = priorityQueue.remove();
            /*++count;*/
            GeographicPoint currVertex = currDistance.getToVertex();
            
            nodeSearched.accept(currVertex); // pass the neighbor vertex to GUI
            
            if (currVertex.equals(goal)) {
                // found the goal vertex
                break;
            }
            
            List<DirectedEdge> neighborList = adjList.get(currVertex);
            if (neighborList != null) {
                
                // perform relax operation on all the outgoing edges
                for (DirectedEdge edge : neighborList) {
                    GeographicPoint neighbor = edge.getEnd();
                    
                    Distance neighborDistance = distances.get(neighbor);
                    double possibleNewDistance = edge.getLength() + currDistance.getDistance();
                    
                    if (Double.compare(possibleNewDistance, neighborDistance.getDistance()) < 0) {
                        // there is a another shorter path
                        neighborDistance.setDistance(possibleNewDistance);
                        parentMap.put(neighbor, currVertex);
                        
                        // enqueue the neighbor
                        priorityQueue.add(neighborDistance);
                    }
                }
            }
        }
        
        /*System.out.println(count);*/
        return constructPath(start, goal, parentMap);
    } 
	
    private Map<GeographicPoint, Distance> initDistances(GeographicPoint start, GeographicPoint goal, Algo type) {

        Map<GeographicPoint, Distance> distances = new HashMap<GeographicPoint, Distance>();

        for (GeographicPoint vertex : adjList.keySet()) {

            double heuristicValue = 0.0;
            switch (type) {
            case DIJKSTRA:
                heuristicValue = 0.0;
                break;
            case ASTAR:
                heuristicValue = vertex.distance(goal);
                break;
            default:
                break;
            }

            distances.put(vertex, new Distance(vertex, Integer.MAX_VALUE, heuristicValue));
        }

        // for source both distance and heuristic value should be zero
        distances.put(start, new Distance(start, 0.0, 0.0));

        return distances;

    }
  
	public static void main(String[] args)
	{
		/*System.out.print("Making a new map...");
		MapGraph theMap = new MapGraph();
		System.out.print("DONE. \nLoading the map...");
		GraphLoader.loadRoadMap("data/testdata/simpletest.map", theMap);
		System.out.println("DONE.");*/
		
		// You can use this method for testing.  
		
		
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
