package frc.robot.libyear;

public class FieldRegions {

    // This is an example of how to define a field region. The coordinates are in meters and are
    // relative to the field coordinate system, from the perspective of the blue alliance station.
    // You can define as many regions as you need for your game strategy.
    // public static PolygonRegion ALLIANCE_ZONE =
    //         new PolygonRegion(
    //                 new Translation2d[] {
    //                     new Translation2d(0, 0),
    //                     new Translation2d(0, 8.042),
    //                     new Translation2d(3.963, 8.042),
    //                     new Translation2d(3.963, 0),
    //                     new Translation2d(0, 0),
    //                 },
    //                 "ALLIANCE_ZONE");

    /**
     * Flips the field regions based of FIELD_SYMMETRY type.
     *
     * @apiNote This does not keep track of Red/Blue
     */
    public static void flipRegions() {
        // You put the name of the region you want to flip here. You can flip as many regions as you
        // want by adding more lines like this one. The regions get fliped when the method is called
        // when the alliance is changed.
        // ALLIANCE_ZONE = AllianceFlipUtil.apply(ALLIANCE_ZONE);
    }
}
