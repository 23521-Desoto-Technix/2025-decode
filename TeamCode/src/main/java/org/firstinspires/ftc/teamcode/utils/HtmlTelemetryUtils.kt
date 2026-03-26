package org.firstinspires.ftc.teamcode.utils

/**
 * Utility class for creating formatted HTML telemetry strings with backgrounds and padding.
 * Provides convenience methods for common formatting patterns used in FTC telemetry display.
 */
object HtmlTelemetryUtils {
    const val NBSP = "&nbsp;&nbsp;"

    /**
     * Creates an HTML span with a background color and text padded with non-breaking spaces.
     * @param text The text to display
     * @param backgroundColor The CSS background color (e.g., "#FF0000", "yellow")
     * @param textColor The CSS text color (default "white")
     * @return Formatted HTML span string
     */
    fun createColoredBadge(
        text: String,
        backgroundColor: String,
        textColor: String = "white"
    ): String {
        return "<span style=\"background-color: $backgroundColor; color: $textColor;\">$NBSP$text$NBSP</span>"
    }

    /**
     * Creates an alliance-specific colored badge. Steady display for RED/BLUE, flashing for UNKNOWN.
     * @param alliance The alliance to display
     * @param flashIntervalMs The interval in milliseconds for flashing UNKNOWN (default 500ms)
     * @return Formatted HTML span string with appropriate colors, flashing for UNKNOWN
     */
    fun createAllianceBadge(alliance: Alliance, flashIntervalMs: Long = 500L): String {
        return when (alliance) {
            Alliance.RED -> createColoredBadge("RED", "#FF0000", "white")
            Alliance.BLUE -> createColoredBadge("BLUE", "#0000FF", "white")
            Alliance.UNKNOWN -> createFlashingBadge("UNKNOWN", "yellow", "black", flashIntervalMs)
        }
    }

    /**
     * Creates a flashing version of an HTML badge that alternates between the styled version and plain text.
     * Useful for warnings and important alerts.
     * @param text The text to display
     * @param backgroundColor The CSS background color
     * @param textColor The CSS text color
     * @param flashIntervalMs The interval in milliseconds for flashing (default 500ms)
     * @return Formatted HTML span string or plain text based on flash timing
     */
    fun createFlashingBadge(
        text: String,
        backgroundColor: String,
        textColor: String = "white",
        flashIntervalMs: Long = 500L
    ): String {
        return if (((System.currentTimeMillis() / flashIntervalMs) % 2).toInt() == 0) {
            createColoredBadge(text, backgroundColor, textColor)
        } else {
            "$NBSP$text$NBSP"
        }
    }
}



