package simtorcs.util

import java.io.File
import java.nio.file.Files
import java.nio.file.Paths

class PathUtil {
    companion object
    {
        val workingDir = Paths.get("").toAbsolutePath().toString() + "/"
        val outputDir = "${workingDir}output/"
        val inputDir = "${workingDir}input/"
        val referenceDir = "${workingDir}reference/"

        init {
            if (!Files.exists(Paths.get(outputDir)))
            {
                Files.createDirectory(Paths.get(outputDir))
            }
            else
            {
                File(outputDir).listFiles().forEach { it.deleteRecursively() }
            }

            if (!Files.exists(Paths.get(inputDir)))
            {
                Files.createDirectory(Paths.get(inputDir))
            }

            if (!Files.exists(Paths.get(referenceDir)))
            {
                Files.createDirectory(Paths.get(referenceDir))
            }
        }
    }
}