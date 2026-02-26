package org.firstinspires.ftc.teamcode.lioncore.system;

import android.os.Environment;

import org.json.JSONObject;

import java.io.BufferedReader;
import java.io.File;
import java.io.FileReader;
import java.io.FileWriter;

public class ConstantsStorage {
    private static final String PATH =
            Environment.getExternalStorageDirectory() + "/FIRST/constants.json";

    private static JSONObject cache = null;

    public static void save(String key, double value) {
        try {
            JSONObject json = loadFromDisk();
            json.put(key, value);
            cache = json;
            FileWriter writer = new FileWriter(PATH);
            writer.write(json.toString(2));
            writer.close();
        } catch (Exception e) {}
    }

    public static double get(String key, double fallback) {
        try {
            return loadCached().getDouble(key);
        } catch (Exception e) {
            return fallback;
        }
    }

    public static boolean has(String key) {
        try {
            return loadCached().has(key);
        } catch (Exception e) {
            return false;
        }
    }

    public static void invalidate() {
        cache = null;
    }

    private static JSONObject loadCached() {
        if (cache == null) cache = loadFromDisk();
        return cache;
    }

    private static JSONObject loadFromDisk() {
        try {
            File file = new File(PATH);
            if (!file.exists()) return new JSONObject();
            BufferedReader reader = new BufferedReader(new FileReader(file));
            StringBuilder sb = new StringBuilder();
            String line;
            while ((line = reader.readLine()) != null) sb.append(line);
            reader.close();
            return new JSONObject(sb.toString());
        } catch (Exception e) {
            return new JSONObject();
        }
    }
}
