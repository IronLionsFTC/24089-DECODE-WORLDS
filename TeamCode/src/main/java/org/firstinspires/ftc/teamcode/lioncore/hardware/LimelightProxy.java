package org.firstinspires.ftc.teamcode.lioncore.hardware;

import fi.iki.elonen.NanoHTTPD;

import java.io.InputStream;
import java.net.HttpURLConnection;
import java.net.URL;
import java.util.Map;

public class LimelightProxy extends NanoHTTPD {

    private final String targetBase;

    public LimelightProxy(int port, String targetBase) {
        super(port);
        this.targetBase = targetBase;
    }

    @Override
    public Response serve(IHTTPSession session) {
        try {
            String urlStr = targetBase + session.getUri();
            if (session.getQueryParameterString() != null) {
                urlStr += "?" + session.getQueryParameterString();
            }

            URL url = new URL(urlStr);
            HttpURLConnection conn = (HttpURLConnection) url.openConnection();
            conn.setRequestMethod(session.getMethod().name());
            conn.setInstanceFollowRedirects(false);

            // Forward request headers
            for (Map.Entry<String, String> header : session.getHeaders().entrySet()) {
                conn.setRequestProperty(header.getKey(), header.getValue());
            }

            conn.connect();

            int status = conn.getResponseCode();
            String contentType = conn.getContentType();
            InputStream body;
            try {
                body = conn.getInputStream();
            } catch (Exception e) {
                body = conn.getErrorStream();
            }

            Response resp = newChunkedResponse(
                    Response.Status.lookup(status),
                    contentType != null ? contentType : "application/octet-stream",
                    body
            );

            // Forward response headers
            for (Map.Entry<String, java.util.List<String>> h : conn.getHeaderFields().entrySet()) {
                if (h.getKey() != null && h.getValue() != null && !h.getValue().isEmpty()) {
                    resp.addHeader(h.getKey(), h.getValue().get(0));
                }
            }

            return resp;

        } catch (Exception e) {
            return newFixedLengthResponse(Response.Status.INTERNAL_ERROR,
                    "text/plain", "Proxy error: " + e);
        }
    }
}
