package jp.jaxa.iss.kibo.rpc.defaultapk;

import gov.nasa.arc.astrobee.types.Quaternion;

class Utilities {
    public static Quaternion eulerToQuaternion(double pitch, double yaw, double roll) {
        double cy = Math.cos(yaw * 0.5);
        double sy = Math.sin(yaw * 0.5);
        double cp = Math.cos(pitch * 0.5);
        double sp = Math.sin(pitch * 0.5);
        double cr = Math.cos(roll * 0.5);
        double sr = Math.sin(roll * 0.5);

        double w = cr * cp * cy + sr * sp * sy;
        double x = sr * cp * cy - cr * sp * sy;
        double y = cr * sp * cy + sr * cp * sy;
        double z = cr * cp * sy - sr * sp * cy;

        return new Quaternion((float)x, (float)y, (float)z, (float)w);
    }

    public static Quaternion multiplyQuaternions(Quaternion q1, Quaternion q2) {
        double x1 = q1.getX(), y1 = q1.getY(), z1 = q1.getZ(), w1 = q1.getW();
        double x2 = q2.getX(), y2 = q2.getY(), z2 = q2.getZ(), w2 = q2.getW();

        double w = w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2;
        double x = w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2;
        double y = w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2;
        double z = w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2;

        return new Quaternion((float)x, (float)y, (float)z, (float)w);
    }


}