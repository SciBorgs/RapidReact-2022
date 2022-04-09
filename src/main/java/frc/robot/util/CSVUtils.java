package frc.robot.util;

import java.io.BufferedReader;
import java.io.File;
import java.io.FileNotFoundException;
import java.io.FileOutputStream;
import java.io.FileReader;
import java.io.PrintWriter;
import java.util.ArrayList;
import java.util.stream.Collectors;

public class CSVUtils {

    public static class CSVWriter {
        private PrintWriter writer;

        public CSVWriter(String path) {
            try {
                this.writer = new PrintWriter(new FileOutputStream(
                        new File(path),
                        true /* append = true */));
            } catch (FileNotFoundException e) {
                this.writer = null;
            }

        }

        public void addData(Object... objects) {
            for (int i = 0; i < objects.length; i++) {
                this.writer.print(objects[i]);
                if (i != objects.length - 1) {
                    this.writer.print(",");
                } else {
                    this.writer.println();
                }
            }
            flush();
        }

        public void flush() {
            this.writer.flush();
        }

        public void close() {
            this.writer.close();
        }
    }

    public static class CSVReader {
        private BufferedReader reader;

        public CSVReader(String path) {
            try {
                this.reader = new BufferedReader(new FileReader(path));
            } catch (FileNotFoundException e) {
                this.reader = null;
            }

        }

        public ArrayList<String[]> readData() {
            return new ArrayList<>(this.reader.lines().map(e -> e.split(",")).collect(Collectors.toList()));
        }
    }

    
    public static CSVWriter newWriter(String path) {
        return new CSVWriter(path);
    }
    
    public static CSVReader newReader(String path) {
        return new CSVReader(path);
    }
}
