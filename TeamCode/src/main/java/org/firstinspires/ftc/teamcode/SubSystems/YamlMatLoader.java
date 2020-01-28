package org.firstinspires.ftc.teamcode.SubSystems;

import java.io.FileReader;
import java.io.FileNotFoundException;
import java.util.List;
import java.util.Map;
import java.util.Scanner;

import org.opencv.core.CvType;
import org.opencv.core.Mat;

import com.esotericsoftware.yamlbeans.YamlException;
import com.esotericsoftware.yamlbeans.YamlReader;

// This code was adopted from https://stackoverflow.com/questions/34297676/how-to-load-opencv-matrices-saved-with-filestorage-in-java
// Modified by Andrew Chiang on 1/25/2020 to support importing a specified matrix
public class YamlMatLoader {
    // This nested class specifies the expected variables in the file
    // Mat cannot be used directly because it lacks rows and cols variables
    protected static class MatStorage {
        public int rows;
        public int cols;
        public String dt;
        public List<String> data;

        // The empty constructor is required by YamlReader
        public MatStorage() {
        }

        public float[] getData() {
            float[] dataOut = new float[data.size()];
            for (int i = 0; i < dataOut.length; i++) {
                dataOut[i] = Float.parseFloat(data.get(i));
            }

            return dataOut;
        }
    }

    // Loading function
    public static Mat getMatYml(String path, String label) {
        try {
            YamlReader reader = new YamlReader(new FileReader(path));

            // Set the tag "opencv-matrix" to process as MatStorage
            // I'm not sure why the tag is parsed as
            // "tag:yaml.org,2002:opencv-matrix"
            // rather than "opencv-matrix", but I determined this value by
            // debugging
            reader.getConfig().setClassTag("tag:yaml.org,2002:opencv-matrix", MatStorage.class);
//            reader.getConfig().setClassTag("opencv-matrix", MatStorage.class);

            // Read the string
            Map map = (Map) reader.read();

            // In file, the variable name for the Mat is "M"
            MatStorage data = (MatStorage) map.get(label);

            // Create a new Mat to hold the extracted data
            Mat m = new Mat(data.rows, data.cols, CvType.CV_32FC1);
            m.put(0, 0, data.getData());
            return m;
        } catch (FileNotFoundException | YamlException e) {
            e.printStackTrace();
        }
        return null;
    }
}