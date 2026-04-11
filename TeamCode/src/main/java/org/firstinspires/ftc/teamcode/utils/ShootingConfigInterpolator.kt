package org.firstinspires.ftc.teamcode.utils

object ShootingConfigInterpolator {
    data class ShootingConfig(val flywheelSpeed: Double, val hoodPosition: Double)

    enum class ShootingZone {
        NEAR,
        FAR,
    }

    private data class ShootingConfigPoint(val centerDistance: Double, val config: ShootingConfig)

    private data class ZoneConfig(
        val minDistance: Double,
        val maxDistance: Double,
        val defaultConfig: ShootingConfig,
        val points: List<ShootingConfigPoint>,
    )

    private val nearZoneConfig =
        ZoneConfig(
            minDistance = 20.0,
            maxDistance = 110.0,
            defaultConfig = ShootingConfig(1_700.0, 0.65),
            points =
                listOf(
                    ShootingConfigPoint(47.5, ShootingConfig(1_400.0, 0.47)),
                    ShootingConfigPoint(60.0, ShootingConfig(1_400.0, 0.58)),
                    ShootingConfigPoint(81.5, ShootingConfig(1_500.0, 0.68)),
                    ShootingConfigPoint(90.5, ShootingConfig(1_600.0, 0.73)),
                    ShootingConfigPoint(95.5, ShootingConfig(1_600.0, 0.78)),
                    ShootingConfigPoint(101.0, ShootingConfig(1_600.0, 0.73)),
                    ShootingConfigPoint(107.0, ShootingConfig(1_700.0, 0.75)),
                ),
        )

    private val farZoneConfig =
        ZoneConfig(
            minDistance = 120.0,
            maxDistance = 135.0,
            defaultConfig = ShootingConfig(1_950.0, 0.9),
            points =
                listOf(
                    ShootingConfigPoint(127.5, ShootingConfig(2_000.0, 0.96)),
                    ShootingConfigPoint(144.0, ShootingConfig(2_050.0, 0.94)),
                    ShootingConfigPoint(150.0, ShootingConfig(2_050.0, 0.92)),
                ),
        )

    private fun lerp(a: Double, b: Double, t: Double): Double {
        return a + (b - a) * t
    }

    private fun interpolate(distance: Double, zone: ZoneConfig): ShootingConfig {
        val sortedPoints = zone.points.sortedBy { it.centerDistance }
        if (distance < zone.minDistance || distance > zone.maxDistance || sortedPoints.isEmpty()) {
            return zone.defaultConfig
        }

        if (distance <= sortedPoints.first().centerDistance) return sortedPoints.first().config
        if (distance >= sortedPoints.last().centerDistance) return sortedPoints.last().config

        for (i in 0 until sortedPoints.lastIndex) {
            val a = sortedPoints[i]
            val b = sortedPoints[i + 1]
            if (distance >= a.centerDistance && distance <= b.centerDistance) {
                val ratio = (distance - a.centerDistance) / (b.centerDistance - a.centerDistance)
                return ShootingConfig(
                    flywheelSpeed = lerp(a.config.flywheelSpeed, b.config.flywheelSpeed, ratio),
                    hoodPosition = lerp(a.config.hoodPosition, b.config.hoodPosition, ratio),
                )
            }
        }

        return zone.defaultConfig
    }

    fun getConfig(distance: Double, zone: ShootingZone): ShootingConfig {
        val zoneConfig = when (zone) {
            ShootingZone.NEAR -> nearZoneConfig
            ShootingZone.FAR -> farZoneConfig
        }
        return interpolate(distance, zoneConfig)
    }
}
