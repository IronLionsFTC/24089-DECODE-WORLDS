package org.firstinspires.ftc.teamcode.lioncore.hardware;

import org.java_websocket.WebSocket;
import org.java_websocket.client.WebSocketClient;
import org.java_websocket.handshake.ClientHandshake;
import org.java_websocket.handshake.ServerHandshake;
import org.java_websocket.server.WebSocketServer;

import java.net.InetSocketAddress;
import java.net.URI;
import java.nio.ByteBuffer;
import java.util.Collections;
import java.util.HashSet;
import java.util.Set;

public class LimelightWebSocketBridge {

    private static LimelightWebSocketBridge instance;

    private final Set<WebSocket> clients = Collections.synchronizedSet(new HashSet<>());
    private WebSocketServer server;

    private LimelightWebSocketBridge(int port, String limelightWsUrl) {
        server = new WebSocketServer(new InetSocketAddress(port)) {

            @Override
            public void onOpen(WebSocket conn, ClientHandshake handshake) {
                clients.add(conn);

                try {
                    // Connect to the Limelight WS for this client
                    WebSocketClient llClient = new WebSocketClient(new URI(limelightWsUrl)) {
                        @Override
                        public void onOpen(ServerHandshake handshakedata) {}

                        @Override
                        public void onMessage(String message) {
                            conn.send(message);  // forward text
                        }

                        @Override
                        public void onMessage(ByteBuffer bytes) {
                            conn.send(bytes);    // forward binary
                        }

                        @Override
                        public void onClose(int code, String reason, boolean remote) {}
                        @Override
                        public void onError(Exception ex) {
                            ex.printStackTrace();
                        }
                    };

                    llClient.connect();
                    conn.setAttachment(llClient);

                } catch (Exception e) {
                    e.printStackTrace();
                }
            }

            @Override
            public void onClose(WebSocket conn, int code, String reason, boolean remote) {
                WebSocketClient c = conn.getAttachment();
                if (c != null) c.close();
                clients.remove(conn);
            }

            @Override
            public void onMessage(WebSocket conn, String message) {
                WebSocketClient c = conn.getAttachment();
                if (c != null) c.send(message);
            }

            @Override
            public void onMessage(WebSocket conn, ByteBuffer message) {
                WebSocketClient c = conn.getAttachment();
                if (c != null) c.send(message);
            }

            @Override
            public void onError(WebSocket conn, Exception ex) {
                ex.printStackTrace();
            }

            @Override
            public void onStart() {
                System.out.println("WebSocket bridge started on port " + port);
            }
        };

        server.start();
    }

    public static void start(int port, String limelightWsUrl) {
        if (instance == null) {
            instance = new LimelightWebSocketBridge(port, limelightWsUrl);
        }
    }
}
