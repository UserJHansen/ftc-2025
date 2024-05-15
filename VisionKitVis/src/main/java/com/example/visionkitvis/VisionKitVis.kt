package com.example.visionkitvis

import com.example.visionkit.Camera
import com.example.visionkit.Line
import com.example.visionkit.Plane
import com.example.visionkit.Point2D
import com.example.visionkit.Point3D
import com.example.visionkit.Renderable
import com.example.visionkit.Rotation3D
import com.example.visionkit.objects.Box
import com.example.visionkit.objects.FieldPerimeter
import com.example.visionkit.objects.PlaneRenderer
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
        Rotation3D(0.0, 0.0, 1.0),
        90.0,
        911.2968158 * 2 / 1920,
        513.6169402 * 2 / 1080
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

        // Draw fps
        g.font = Font("Sans", Font.BOLD, 20)
        g.color = Color.BLUE

        var screenLine = 0
        fun drawLine(text: String) {
            g.drawString(
                text,
                (screenCenter.x - screenRadius).toFloat(),
                (screenCenter.y - screenRadius).toFloat() + 20 + 20 * screenLine
            )
            screenLine++
        }

        drawLine("FPS: %.1f".format(loopManager.fps))
        drawLine("Camera: %s, mag: ".format(camera.position))
        drawLine("Camera: %s".format(camera.direction))
        drawLine("Box: %s".format((objects[0] as Box).position))
        drawLine("Direction dot: %s".format(camera.direction.dot(camera.up)))

        camera.render(objects).forEach {
            g.color = it.colour
            g.drawLine(
                (screenCenter.x + screenRadius * it.p1.x).toInt(),
                (screenCenter.y + screenRadius * it.p1.y).toInt(),
                (screenCenter.x + screenRadius * it.p2.x).toInt(),
                (screenCenter.y + screenRadius * it.p2.y).toInt()
            )
        }

        val labels = objects.flatMap { it.labels }

        for (label in labels) {
            val position = label.position

            if ((position - camera.position).dot(camera.direction) < 0.0) {
                continue
            }

            val screenPosition = camera.convertToScreenCoords(label.position)
            g.color = Color.RED

            g.drawString(
                label.text,
                (screenCenter.x + screenRadius * screenPosition.x).toInt(),
                (screenCenter.y + screenRadius * screenPosition.y).toInt()
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

                val axis = camera.up.cross(camera.direction)
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
                    KeyEvent.VK_A -> camera.position -= camera.up.cross(camera.direction) * moveSpeed
                    KeyEvent.VK_D -> camera.position += camera.up.cross(camera.direction) * moveSpeed
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

        objects.add(Box(Point3D(0.0, 0.0, 0.0)))

        objects.add(
            PlaneRenderer(
                Plane(
                    Point3D(0.0, 10.0, 0.0),
                    Rotation3D(0.0, 0.0, 1.0),
                    Rotation3D(0.0, 1.0, 0.0)
                ),
                listOf(
                    Line(Point2D(-1.0, -1.0), Point2D(1.0, -1.0)),
                    Line(Point2D(-1.0, 1.0), Point2D(1.0, 1.0)),
                    Line(Point2D(-1.0, -1.0), Point2D(-1.0, 1.0)),
                    Line(Point2D(1.0, -1.0), Point2D(1.0, 1.0))
                )
            )
        )

        objects.add(
            PlaneRenderer(
                Plane(
                    Point3D(0.0, -10.0, 0.0),
                    Rotation3D(0.0, 0.0, 1.0),
                    Rotation3D(0.0, 1.0, 0.0)
                ),
                listOf(
                    Line(Point2D(-1.0, 1.0), Point2D(-1.0, 2.0)),
                    Line(Point2D(1.0, 1.0), Point2D(1.0, 2.0)),

//                Smile!
                    Line(Point2D(-1.0, -1.0), Point2D(-0.5, -1.5)),
                    Line(Point2D(-0.5, -1.5), Point2D(0.5, -1.5)),
                    Line(Point2D(0.5, -1.5), Point2D(1.0, -1.0))
                ),
                "Smile!"
            )
        )

        objects.add(FieldPerimeter())

        loopManager.start()

        return this
    }
}
