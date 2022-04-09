package frc.robot.util;

import java.io.BufferedReader;
import java.io.File;
import java.io.FileNotFoundException;
import java.io.FileOutputStream;
import java.io.FileReader;
import java.io.IOException;
import java.io.PrintWriter;
import java.util.ArrayList;
import java.util.stream.Collectors;

public class CSVUtils {

    public static class CSVWriter {
        private PrintWriter writer;

        public CSVWriter(String path) {
            File filepath = new File(path);
            boolean fileStatus = false;
            try {
                fileStatus = filepath.createNewFile();
            } catch (IOException e) {
                fileStatus = false;
            }

            if (fileStatus) {

                try {
                    this.writer = new PrintWriter(new FileOutputStream(
                            filepath,
                            true /* append = true */));
                } catch (FileNotFoundException e) {
                    this.writer = null;
                }
            }

        }

        public void addData(Object... objects) {
            if (this.writer != null) {
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
        }

        public void flush() {
            if (this.writer != null)
                this.writer.flush();
        }

        public void close() {
            if (this.writer != null)
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
            if (this.reader != null) {
                return new ArrayList<>(this.reader.lines().map(e -> e.split(",")).collect(Collectors.toList()));
            }
            return null;

        }
    }

    public static CSVWriter newWriter(String path) {
        return new CSVWriter(path);
    }

    public static CSVReader newReader(String path) {
        return new CSVReader(path);
    }
}
