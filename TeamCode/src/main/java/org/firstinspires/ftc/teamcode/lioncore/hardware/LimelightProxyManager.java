package org.firstinspires.ftc.teamcode.lioncore.hardware;

import fi.iki.elonen.NanoHTTPD;

public class LimelightProxyManager {

    private static LimelightProxy httpProxy;
    private static boolean started = false;

    public static void start() {
        if (started) return;

        try {
            httpProxy = new LimelightProxy(5808, "http://172.29.0.1:5801");
            httpProxy.start(NanoHTTPD.SOCKET_READ_TIMEOUT, false);
            System.out.println("HTTP proxy started on port 5808");

            LimelightWebSocketBridge.start(5809, "ws://172.29.0.1:5801/ws");

            started = true;
        } catch (Exception e) {
            e.printStackTrace();
        }
    }
}
