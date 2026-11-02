package org.firstinspires.ftc.teamcode.logging

import java.io.BufferedWriter
import java.io.File
import java.io.FileWriter
import java.io.IOException
import java.text.SimpleDateFormat
import java.util.Date
import java.util.Locale

/**
 * Writes a table to a .csv file, which opens in Google Sheets, Excel or Python (pandas).
 * The first line holds the column names, and every addRow() adds one line under them.
 *
 * Each run gets its own file, named with the date and time, for example shots_2026-10-22_17-05-31.csv.
 * If anything goes wrong with the file (for example the storage is full), logging stops quietly:
 * a logging problem must never stop the robot.
 */
class CsvFile(folder: File, namePrefix: String, private val columns: List<String>) {
    /** The file we write to (null if it could not be created). */
    var file: File? = null
        private set
    private var writer: BufferedWriter? = null

    init {
        try {
            folder.mkdirs()
            val time = SimpleDateFormat("yyyy-MM-dd_HH-mm-ss", Locale.US).format(Date())
            var newFile = File(folder, "${namePrefix}_$time.csv")
            var copy = 2
            while (newFile.exists()) {
                // Two runs in the same second: do not overwrite the first one.
                newFile = File(folder, "${namePrefix}_${time}_$copy.csv")
                copy += 1
            }
            writer = BufferedWriter(FileWriter(newFile))
            file = newFile
            writeLine(columns)
        } catch (e: IOException) {
            writer = null
        }
    }

    /** True if the file is open and we are writing to it. */
    fun isWorking(): Boolean = writer != null

    /** Adds one row. Give one value for each column, in the same order as the column names. */
    fun addRow(values: List<Any?>) {
        if (values.size != columns.size) {
            throw IllegalArgumentException("Got ${values.size} values for ${columns.size} columns")
        }
        writeLine(values.map { format(it) })
    }

    /** Makes sure everything written so far is really in the file. */
    fun flush() {
        try {
            writer?.flush()
        } catch (e: IOException) {
            writer = null
        }
    }

    /** Call this when the OpMode stops. */
    fun close() {
        try {
            writer?.close()
        } catch (e: IOException) {
            // Nothing more we can do.
        }
        writer = null
    }

    private fun writeLine(cells: List<String>) {
        val out = writer ?: return
        try {
            out.write(cells.joinToString(","))
            out.newLine()
        } catch (e: IOException) {
            writer = null
        }
    }

    /**
     * Turns one value into text. Numbers always use a dot (2.5, never 2,5), whatever language the
     * Control Hub is set to. Yes/no values become 1/0. Empty cells mean "unknown".
     */
    private fun format(value: Any?): String {
        return when (value) {
            null -> ""
            is Double -> if (value.isNaN() || value.isInfinite()) "" else String.format(Locale.US, "%.3f", value)
            is Float -> format(value.toDouble())
            is Boolean -> if (value) "1" else "0"
            else -> value.toString().replace(",", ";")   // a comma would split the cell in two
        }
    }
}
