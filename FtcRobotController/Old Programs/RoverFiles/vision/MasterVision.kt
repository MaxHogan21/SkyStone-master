package org.firstinspires.ftc.teamcode.vision

import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.robotcore.external.ClassFactory
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer


class MasterVision(private val parameters: VuforiaLocalizer.Parameters, val hMap: HardwareMap, val tfLiteAlgorithm: TFLiteAlgorithm) : Thread() {
    var vuforiaLocalizer: VuforiaLocalizer? = null
    val tfLite = TFLite(this)

    enum class TFLiteAlgorithm{
        INFER_NONE
    }


    fun init() {
        if (vuforiaLocalizer == null)
            vuforiaLocalizer = ClassFactory.getInstance().createVuforia(parameters)
        tfLite.init()
    }

    fun enable() {
        init()
        tfLite.enable()
    }

    fun disable() {
        tfLite.disable()
    }

    fun shutdown() {
        disable()
        tfLite.shutdown()
    }

    override fun run() {
        try {
            while (true) {
                tfLite.updateSampleOrder()
            }
        } catch (ex: InterruptedException) {
            Thread.currentThread().interrupt()
        }
    }

    init {
        start()
    }
}

