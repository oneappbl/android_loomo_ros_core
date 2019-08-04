package org.loomo.ros;

import org.ros.message.Duration;
import org.ros.message.Time;

/**
 * Created by kai on 17-11-2.
 */

public class Utils {
    private static final String TAG = "Utils";

    public static double platformStampToSecond(long stamp) {
        return (double)stamp/1.0E6D;
    }

    public static long platformStampInMillis(long stamp) {
        return (long)((double)stamp/1.0E3D);
    }

    public static long platformStampInNano(long stamp) {
        return (stamp * 1000);
    }

}
