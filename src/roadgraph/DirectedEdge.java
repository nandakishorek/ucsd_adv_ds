/**
 * 
 */
package roadgraph;

import geography.GeographicPoint;

/**
 * Represents a directed edge from 'start' vertex to 'end' vertex with a length 'length'
 * and type 'type'
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
    
    public DirectedEdge(GeographicPoint start, GeographicPoint end, String name, String type, double length) {
        super();
        this.start = start;
        this.end = end;
        this.name = name;
        this.type = type;
        this.length = length;
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
        return type;
    }

    public double getLength() {
        return length;
    }

    @Override
    public String toString() {
        return start.toString() + " " + end.toString() + " \"" + name + "\"" + type + " " + length;
    }

}
