package jp.jaxa.iss.kibo.rpc.defaultapk;

import java.util.HashMap;
import java.util.Map;

import gov.nasa.arc.astrobee.types.Point;
import gov.nasa.arc.astrobee.types.Quaternion;

public class Area {
    public static final Point AREA_1 = new Point(10.9, -9.92284, 5.195);
    public static final Quaternion AREA_1_QUATERNION = new Quaternion(0f, 0f, -0.707f, 0.707f);
    public static final Point AREA_2 = new Point(10.93, -8.88, 4.4);
    public static final Quaternion AREA_2_QUATERNION = new Quaternion(0, 0.707f, 0, 0.707f);
    public static final Point AREA_3 = new Point(10.93, -7.93, 4.4);
    public static final Quaternion AREA_3_QUATERNION = new Quaternion(0, 0.707f, 0, 0.707f);
    public static final Point AREA_4 = new Point(10.5, -7, 4.92); // changed this
    public static final Quaternion AREA_4_QUATERNION = new Quaternion(0, 1, 0, 0);
    public static final Point ASTRONAUT_POINT = new Point(11.143, -6.7607, 4.9654);
    public static final Quaternion ASTRONAUT_ORIENTATION = new Quaternion(0, 0, 0.707f, 0.707f);

    public static final Map<String, TreasureLocation> treasureLocations = new HashMap<>();

    public static class TreasureLocation {
        public Point treasurePoint;
        public Quaternion treasureQuaternion;
        public TreasureLocation(Point point, Quaternion quaternion) {
            treasurePoint = point;
            treasureQuaternion = quaternion;
        }
    }
}
