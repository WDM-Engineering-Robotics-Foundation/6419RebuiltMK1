package frc.robot.vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rectangle2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.util.struct.Struct;

public class FuelClump {
    
    public static final Struct<FuelClump> struct = new FuelClumpStruct();
    
    private final Translation2d centroid;

    private final Rectangle2d bounds;

    private final int area;

    private final int id;

    public FuelClump(Translation2d centroid, Rectangle2d bounds, int area, int id) {
        this.centroid = centroid;
        this.bounds = bounds;
        this.area = area;
        this.id = id;
    }

    public Translation2d getCentroid() {
        return centroid;
    }

    public Rectangle2d getBounds() {
        return bounds;
    }

    public int getArea() {
        return area;
    }

    public int getId() {
        return id;
    }
}
