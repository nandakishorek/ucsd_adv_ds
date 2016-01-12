/**
 * 
 */
package roadgraph;

import geography.GeographicPoint;

/**
 * Represents a directed edge from 'start' vertex to 'end' vertex with a length
 * 'length' and type 'type'
 * 
 * @author kishore
 *
 */
public class DirectedEdge {

    private GeographicPoint start;
    private GeographicPoint end;
    private String name;
    private String type;
    private double length;
    private double travelTime;

    public DirectedEdge(GeographicPoint start, GeographicPoint end, String name, String type, double length) {
        super();
        this.start = start;
        this.end = end;
        this.name = name;
        this.length = length;
        
        double speed = RoadType.valueOf(type.toUpperCase()).getSpeed();
        this.travelTime = length / speed;
    }

    public GeographicPoint getStart() {
        return start;
    }

    public GeographicPoint getEnd() {
        return end;
    }

    public String getName() {
        return name;
    }

    public String getType() {
        return type.toString().toLowerCase();
    }

    public double getLength() {
        return length;
    }

    public double getTravelTime() {
        return travelTime;
    }

    @Override
    public String toString() {
        return start.toString() + " " + end.toString() + " \"" + name + "\"" + type + " " + length;
    }

}
