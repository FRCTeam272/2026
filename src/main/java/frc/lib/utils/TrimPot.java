package frc.lib.utils;

public class TrimPot {
    public String name;
    public double adjusterValue;

    public TrimPot(String name) {
        this.name = name;
        try {
            java.nio.file.Path filePath = java.nio.file.Paths.get(name + ".trimpot");
            java.util.List<String> lines = java.nio.file.Files.readAllLines(filePath);
            adjusterValue = Double.parseDouble(lines.get(0));
        } catch (java.io.IOException e) {
            adjusterValue = 0.0;
            System.err.println("Error reading file: " + e.getMessage());
        }
    }

    public void saveAdjusterValue() {
        try {
            java.nio.file.Path filePath = java.nio.file.Paths.get(name + ".trimpot");
            java.nio.file.Files.write(filePath, String.valueOf(adjusterValue).getBytes());
        } catch (java.io.IOException e) {
            System.err.println("Error writing file: " + e.getMessage());
        }
    }
}
