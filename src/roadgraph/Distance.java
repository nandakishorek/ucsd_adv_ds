/**
 * 
 */
package roadgraph;

import geography.GeographicPoint;

/**
 * The class representing an entry in the priority queue in Dijsktra's algo
 * 
 * @author kishore
 *
 */
public class Distance implements Comparable<Distance> {
    
    // any vertex other than source
    private GeographicPoint toVertex;
    
    // the shortest distance from the source vertex, same as the shortest path length
    private double distance;
    
    // the displacement between goal vertex and this vertex
    private double distToGoal;

    public Distance(GeographicPoint toVertex, double distance, double distToGoal) {
        super();
        this.toVertex = toVertex;
        this.distance = distance;
        this.distToGoal = distToGoal;
    }

    /**
     * Method to compare priority of one vertex to another in Dijkstra's algo
     * Vertex closer to the source get higher priority i.e., lower distance will get higher priority.
     * 
     * @param other The other vertex to be compared with
     */
    @Override
    public int compareTo(Distance other) {
        
        if ((this.distance + this.distToGoal) < (other.distance + other.distToGoal)) {
            return -1;
        } else if ((this.distance + this.distToGoal)  > (other.distance + other.distToGoal)) {
            return 1;
        }
        
        return 0;
    }

    public GeographicPoint getToVertex() {
        return toVertex;
    }

    public double getDistance() {
        return distance;
    }

    public void setToVertex(GeographicPoint toVertex) {
        this.toVertex = toVertex;
    }

    public void setDistance(double distance) {
        this.distance = distance;
    }

    @Override
    public String toString() {
        return "Distance [toVertex=" + toVertex + ", distance=" + distance + "]";
    }
    
    
    
}
