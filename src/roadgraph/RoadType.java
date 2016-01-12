package roadgraph;

public enum RoadType {
    MOTORWAY (70.00),
    TRUNK (60.00),
    PRIMARY (50.00),
    SECONDARY (45.00),
    TERTIARY (40.00),
    MOTORWAY_LINK (40.00),
    TRUNK_LINK (40.00),
    PRIMARY_LINK (30.00),
    SECONDARY_LINK (30.00),
    TERTIARY_LINK (30.00),
    LIVING_STREET (25.00),
    RESIDENTIAL (25.00),
    UNCLASSIFIED (25.00);
    
    private final double speed;
    
    private RoadType(double speed) {
        this.speed = speed;
    }
    
    public double getSpeed() {
        return speed;
    }
    
}
