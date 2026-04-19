package frc.robot.vision;

import java.nio.ByteBuffer;

import edu.wpi.first.math.geometry.Rectangle2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.util.struct.Struct;

class FuelClumpStruct implements Struct<FuelClump> {

    @Override
    public Class<FuelClump> getTypeClass() {
        return FuelClump.class;
    }

    @Override
    public String getTypeName() {
        return "FuelClump";
    }

    @Override
    public int getSize() {
        return Translation2d.struct.getSize() + Rectangle2d.struct.getSize() + kSizeInt32 + kSizeInt16;
    }

    @Override
    public String getSchema() {
        
        return "Translation2d centroid; Rectangle2d bounds; int32 area; uint16 id";
    }

    @Override
    public FuelClump unpack(ByteBuffer bb) {
        return new FuelClump(
            Translation2d.struct.unpack(bb), 
            Rectangle2d.struct.unpack(bb),
            bb.getInt(),
            Short.toUnsignedInt(bb.getShort())
        );
    }

    @Override
    public void pack(ByteBuffer bb, FuelClump value) {
        Translation2d.struct.pack(bb, value.getCentroid());
        Rectangle2d.struct.pack(bb, value.getBounds());
        bb.putInt(value.getArea());
        bb.putShort((short) value.getId());
    }
    
}
