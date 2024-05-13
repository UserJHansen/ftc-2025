package com.example.visionkitvis

import com.example.visionkit.Camera
import com.example.visionkit.Line
import com.example.visionkit.Point2D
import com.example.visionkit.Point3D
import com.example.visionkit.Rotation3D
import com.example.visionkitvis.ui.WindowFrame
import com.example.visionkitvis.ui.colorscheme.ColorManager
import java.awt.Color
import java.awt.Font
import java.awt.Graphics2D
import java.awt.RenderingHints
import java.awt.Robot
import java.awt.event.KeyEvent
import java.awt.event.KeyListener
import java.awt.event.MouseEvent
import java.awt.event.MouseMotionListener
import javax.swing.UIManager
import kotlin.math.atan
import kotlin.math.cos
import kotlin.math.sin


open class VisionKitVis(windowSize: Int, fps: Int = 60) {
    val windowFrame = WindowFrame("Vision Kit", windowSize)
    val canvas = windowFrame.canvas

    private val colorManager = ColorManager()

    private val objects = mutableListOf<Renderable>()
    val camera = Camera(
        Point3D(0.0, 0.0, 0.0),
        Rotation3D(0.0, 1.0, 0.0),
        Rotation3D(0.0,0.0,1.0),
        90.0
    )
    var mousePosition: Point2D = Point2D(0.0, 0.0)

    private val render: () -> Unit = {
        val g = canvas.bufferStrat.drawGraphics as Graphics2D
        g.setRenderingHint(RenderingHints.KEY_ANTIALIASING, RenderingHints.VALUE_ANTIALIAS_ON)

//            Constrain to 16:9, using the smallest dimension, and making the rest black
        val aspectRatio = 1920.0/1080.0
        val screenWidth = canvas.width
        val screenHeight = canvas.height
        val screenAspectRatio = screenWidth.toDouble()/screenHeight.toDouble()

        g.color = Color.BLACK
        g.fillRect(0, 0, screenWidth, screenHeight)
        g.color = Color.WHITE
        val screenCenter = Point2D(screenWidth/2.0, screenHeight/2.0)
        val screenRadius = if (screenAspectRatio > aspectRatio) screenHeight/3.0 else screenWidth/3.0

        g.fillRect(
            (screenCenter.x - screenRadius).toInt(),
            (screenCenter.y - screenRadius).toInt(),
            (2*screenRadius).toInt(),
            (2*screenRadius).toInt()
        )

        val fx = 911.2968158*2/1920
        val fy = 513.6169402*2/1080

        fun convertToScreenCoords(point: Point3D): Point2D {
            var p = point - camera.position
            p = Point3D(
                p.dot(camera.direction),
                p.dot(camera.up.cross(camera.direction)),
                p.dot(camera.up)
            )

            p.y = atan(p.y/p.x)*Math.PI*fx
            p.z = atan(p.z/p.x)*Math.PI*fy

            return Point2D(
                (screenCenter.x + screenRadius*p.y),
                (screenCenter.y + screenRadius*p.z)
            )
        }

        // Draw fps
        g.font = Font("Sans", Font.BOLD, 20)
        g.color = Color.BLUE

        var screenLine = 0
        fun drawLine(text: String) {
            g.drawString(text, (screenCenter.x-screenRadius).toFloat(), (screenCenter.y-screenRadius).toFloat()+20+20*screenLine)
            screenLine++
        }

        drawLine("FPS: %.1f".format(loopManager.fps))
        drawLine("Camera: %s, mag: ".format(camera.position))
        drawLine("Camera: %s".format(camera.direction))
        drawLine("Camera: %s".format(camera.up))
        drawLine("Box: %s".format((objects[0] as Box).position))
        drawLine("Direction dot: %s".format(camera.direction.dot(camera.up)))

        val lines = mutableListOf<Line>()
        val labels = mutableListOf<Label>()

        for (obj in objects) {
            lines.addAll(obj.render(camera))
            labels.add(obj.label)
        }

        labels.add(
            Label(
                "Circle",
                Point3D(0.0, 0.0, 0.0)
            )
        )

        val circle = mutableListOf<Line>()
        for (i in 0..100) {
            val startAngle = i.toDouble()/100.0*2.0*Math.PI
            val finishAngle = (i+1).toDouble()/100.0*2.0*Math.PI
            val p1 = Point3D(cos(startAngle), sin(startAngle), 0.0)
            val p2 = Point3D(cos(finishAngle), sin(finishAngle), 0.0)
            circle.add(Line(p1, p2))
        }

        lines.addAll(circle)

        for (label in labels) {
            val position = label.position - camera.position

            if (position.dot(camera.direction) < 0.0) {
                continue
            }

            val screenPosition = convertToScreenCoords(label.position)
            g.color = Color.RED

            g.drawString(
                label.text,
                screenPosition.x.toInt(),
                screenPosition.y.toInt()
            )
        }

        for (line in lines) {
            val p1 = line.p1 - camera.position
            val p2 = line.p2 - camera.position

//            Convert the 3d points onto the camera, using the camera's position, direction and fov

            if (p1.dot(camera.direction) < 0.0 && p2.dot(camera.direction) < 0.0) {
                g.color = Color.RED
                continue
            } else if (p1.angleBetween(p2) > Math.toRadians(camera.fov)) {
                g.color = Color.GREEN
                continue
            } else if (p1.angleBetween(camera.direction) > Math.toRadians(camera.fov)) {
                g.color = Color.BLUE
                continue
            } else if (p2.angleBetween(camera.direction) > Math.toRadians(camera.fov)) {
                g.color = Color.PINK
                continue
            } else {
                g.color = Color.BLACK
            }

            val screen_p1 = convertToScreenCoords(line.p1)
            val screen_p2 = convertToScreenCoords(line.p2)

            g.drawLine(
                screen_p1.x.toInt(),
                screen_p1.y.toInt(),
                screen_p2.x.toInt(),
                screen_p2.y.toInt()
            )
        }

        g.dispose()
        canvas.bufferStrat.show()
    }

    private val update: (deltaTime: Long) -> Unit = { deltaTime ->
        (objects[0] as Box).position.x = sin(System.currentTimeMillis()/1000.0)
        (objects[0] as Box).position.y = sin(System.currentTimeMillis()/1000.0) + 10
        (objects[0] as Box).position.z = cos(System.currentTimeMillis()/1000.0)
    }

    private val loopManager = LoopManager(fps, update, render)

    init {
        // Core init
        UIManager.setLookAndFeel(UIManager.getSystemLookAndFeelClassName())

        windowFrame.contentPane.background = colorManager.theme.UI_MAIN_BG
        windowFrame.canvasPanel.background = colorManager.theme.UI_MAIN_BG

        windowFrame.pack()

        canvas.addMouseMotionListener(object : MouseMotionListener {
            val robot = Robot()

            override fun mouseDragged(p0: MouseEvent?) {}

            override fun mouseMoved(e: MouseEvent) {
                val mouseDelta = Point2D(
                    (e.x.toDouble() - mousePosition.x) / canvas.width,
                    (e.y.toDouble() - mousePosition.y) / canvas.height
                )
                robot.mouseMove(windowFrame.x + canvas.width / 2, windowFrame.y + canvas.height / 2)
                mousePosition =
                    Point2D(canvas.mousePosition.x.toDouble(), canvas.mousePosition.y.toDouble())

                camera.direction = camera.direction.rotateAround(camera.up, mouseDelta.x)

                val axis = camera.direction.cross(camera.up)
                camera.direction = camera.direction.rotateAround(axis, mouseDelta.y)
                camera.up = camera.up.rotateAround(axis, mouseDelta.y)

            }
        })

        canvas.addKeyListener(object : KeyListener {
            override fun keyTyped(p0: KeyEvent?) {}

            val moveSpeed = 0.1
            override fun keyPressed(e: KeyEvent) {
                when (e.keyCode) {
                    KeyEvent.VK_W -> camera.position += camera.direction * moveSpeed
                    KeyEvent.VK_S -> camera.position -= camera.direction * moveSpeed
                    KeyEvent.VK_A -> camera.position += camera.direction.cross(camera.up) * moveSpeed
                    KeyEvent.VK_D -> camera.position -= camera.direction.cross(camera.up) * moveSpeed
                    KeyEvent.VK_Q -> camera.position += camera.up * moveSpeed
                    KeyEvent.VK_E -> camera.position -= camera.up * moveSpeed
                    KeyEvent.VK_R -> {
                        camera.position = Point3D(0.0, 0.0, 0.0)
                        camera.direction = Rotation3D(0.0, 1.0, 0.0)
                        camera.up = Rotation3D(0.0, 0.0, 1.0)
                    }
                }
            }

            override fun keyReleased(p0: KeyEvent?) {}
        })
    }

    fun start(): VisionKitVis {
        windowFrame.isVisible = true

        objects.add(Box())

        loopManager.start()

        return this
    }
}
