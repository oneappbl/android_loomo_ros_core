package org.loomo.ros;

/**
 * Created by kai on 17-7-16.
 * Modified by mfe on 10-9-18.
 */

public interface LoomoRosBridgeConsumer {
    void node_started(LoomoRosBridgeNode mBridgeNode);
    void start();
    void stop();
}
