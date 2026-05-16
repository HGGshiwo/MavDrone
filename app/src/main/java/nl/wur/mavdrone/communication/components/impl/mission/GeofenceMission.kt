package nl.wur.mavdrone.communication.components.impl.mission

import java.io.IOException
import java.lang.ref.WeakReference
import javax.xml.parsers.DocumentBuilderFactory
import javax.xml.parsers.ParserConfigurationException
import kotlin.math.roundToInt

import org.w3c.dom.Element
import org.xml.sax.SAXException

import org.locationtech.jts.geom.Coordinate
import org.locationtech.jts.geom.GeometryFactory
import org.locationtech.jts.geom.Point
import org.locationtech.jts.geom.Polygon

import dji.sdk.keyvalue.value.common.LocationCoordinate3D

import io.reactivex.rxjava3.core.Completable
import io.reactivex.rxjava3.core.Single

import io.dronefleet.mavlink.common.MavCmd
import io.dronefleet.mavlink.common.MavFrame
import io.dronefleet.mavlink.common.MavMissionType
import io.dronefleet.mavlink.common.MissionItemInt

import nl.wur.mavdrone.communication.MavConnection
import nl.wur.mavdrone.communication.components.impl.MissionManager
import nl.wur.mavdrone.communication.utils.DJIMavlinkConversion.Companion.missionItemToCoordinate
import nl.wur.mavdrone.communication.utils.DJIMavlinkConversion.Companion.missionItemToCoordinate3D
import nl.wur.mavdrone.communication.utils.FileUtils.Companion.writeFileUsingTemplate
import nl.wur.mavdrone.communication.utils.KMZ
import nl.wur.mavdrone.utils.CoordinateUtils.Companion.getDistanceInMeterFromTwoGPSLocations
import nl.wur.mavdrone.utils.RxException.FenceMissionConverterException
import nl.wur.mavdrone.utils.RxException.MissionConverterException


class GeofenceMission(
    connectionWR: WeakReference<MavConnection>,
    missionManagerWR: WeakReference<MissionManager>
) : BaseMission(connectionWR, missionManagerWR, "MavGeofenceMission") {
    private val fenceItems: MutableList<FenceItem> = ArrayList()
    var returnPoint: Coordinate? = null
        private set

    override fun toString(): String {
        return "Fence with ${fenceItems.size} fence items ${if (returnPoint == null) "without" else "with"} return point"
    }

    override fun copy(): GeofenceMission {
        val copy = GeofenceMission(connectionWR, missionManagerWR)

        copy.missionItems.addAll(missionItems)
        copy.fenceItems.addAll(fenceItems)
        copy.returnPoint = returnPoint
        return copy
    }

    override fun clear() {
        super.clear()

        fenceItems.clear()
        returnPoint = null
    }

    override fun convertFromMavlink(): Completable {
        fenceItems.clear()
        returnPoint = null

        val missionItemsCopy: List<MissionItemInt> = ArrayList(missionItems)

        return Completable.fromCallable {
            val vertexIncludePoints: MutableList<Coordinate> = ArrayList()
            val vertexExcludePoints: MutableList<Coordinate> = ArrayList()

            var coordinate: Coordinate
            var polygon: FencePolygon
            var circle: FenceCircle

            for (i in missionItemsCopy.indices) {
                val missionItem = missionItemsCopy[i]

                when (missionItem.command().entry()) {
                    MavCmd.MAV_CMD_NAV_FENCE_RETURN_POINT -> this.returnPoint =
                        missionItemToCoordinate3D(missionItem)

                    MavCmd.MAV_CMD_NAV_FENCE_CIRCLE_INCLUSION -> {
                        coordinate = missionItemToCoordinate(missionItem)
                        circle = FenceCircle(coordinate, missionItem.param1().toDouble(), true)
                        fenceItems.add(circle)
                    }

                    MavCmd.MAV_CMD_NAV_FENCE_CIRCLE_EXCLUSION -> {
                        coordinate = missionItemToCoordinate(missionItem)
                        circle = FenceCircle(coordinate, missionItem.param1().toDouble(), false)
                        fenceItems.add(circle)
                    }

                    MavCmd.MAV_CMD_NAV_FENCE_POLYGON_VERTEX_INCLUSION -> {
                        coordinate = missionItemToCoordinate(missionItem)
                        vertexIncludePoints.add(coordinate)

                        if (missionItem.param1().roundToInt() == vertexIncludePoints.size) {
                            // Copy start polygon to make closed linestring
                            vertexIncludePoints.add(vertexIncludePoints[0])

                            polygon =
                                FencePolygon(vertexIncludePoints.toTypedArray<Coordinate>(), true)
                            fenceItems.add(polygon)
                            vertexIncludePoints.clear()
                        }
                    }

                    MavCmd.MAV_CMD_NAV_FENCE_POLYGON_VERTEX_EXCLUSION -> {
                        coordinate = missionItemToCoordinate(missionItem)
                        vertexExcludePoints.add(coordinate)

                        if (missionItem.param1().roundToInt() == vertexExcludePoints.size) {
                            // Copy start polygon to make closed linestring
                            vertexExcludePoints.add(vertexExcludePoints[0])

                            polygon =
                                FencePolygon(vertexExcludePoints.toTypedArray<Coordinate>(), false)
                            fenceItems.add(polygon)
                            vertexExcludePoints.clear()
                        }
                    }

                    else -> throw FenceMissionConverterException(
                        "Cannot convert mission item of type " + missionItem.command()
                            .entry().name + " to geofence!"
                    )
                }
            }

            if (vertexIncludePoints.isNotEmpty() || vertexExcludePoints.isNotEmpty()) {
                throw FenceMissionConverterException("Could not correctly parse mission items: some polygons are not complete!")
            }
        }
    }

    override fun convertToMavlink(targetSystem: Int, targetComponent: Int): Completable {
        missionItems.clear()

        return Completable.fromCallable {
            returnPoint?.let {
                val missionItem = MissionItemInt.builder()
                    .seq(missionItems.size)
                    .frame(MavFrame.MAV_FRAME_GLOBAL_INT)
                    .command(MavCmd.MAV_CMD_NAV_FENCE_RETURN_POINT)
                    .x((it.x * 1e7).toInt())
                    .y((it.y * 1e7).toInt())
                    .z(it.z.toFloat())
                    .targetSystem(targetSystem)
                    .targetComponent(targetComponent)
                    .missionType(MavMissionType.MAV_MISSION_TYPE_FENCE)
                    .build()
                missionItems.add(missionItem)
            }

            for (fenceItem in fenceItems) {
                if (fenceItem is FencePolygon) {
                    val vertices = fenceItem.vertices
                    val inclusion = fenceItem.isInclusion

                    // Skip last vertex because JTS expect them to be closed
                    for (i in 0 until vertices.size - 1) {
                        val missionItem = MissionItemInt.builder()
                            .seq(missionItems.size)
                            .frame(MavFrame.MAV_FRAME_GLOBAL_INT)
                            .command(if (inclusion) MavCmd.MAV_CMD_NAV_FENCE_POLYGON_VERTEX_INCLUSION else MavCmd.MAV_CMD_NAV_FENCE_POLYGON_VERTEX_EXCLUSION)
                            .current(0)
                            .param1((vertices.size - 1).toFloat())
                            .x((vertices[i].x * 1e7).toInt())
                            .y((vertices[i].y * 1e7).toInt())
                            .targetSystem(targetSystem)
                            .targetComponent(targetComponent)
                            .missionType(MavMissionType.MAV_MISSION_TYPE_FENCE)
                            .build()
                        missionItems.add(missionItem)
                    }
                } else if (fenceItem is FenceCircle) {
                    val center = fenceItem.getCenter()
                    val radius = fenceItem.radius
                    val inclusion = fenceItem.isInclusion

                    val missionItem = MissionItemInt.builder()
                        .seq(missionItems.size)
                        .frame(MavFrame.MAV_FRAME_GLOBAL_INT)
                        .command(if (inclusion) MavCmd.MAV_CMD_NAV_FENCE_CIRCLE_INCLUSION else MavCmd.MAV_CMD_NAV_FENCE_CIRCLE_EXCLUSION)
                        .current(0)
                        .param1(radius.toFloat())
                        .x((center.x * 1e7).toInt())
                        .y((center.y * 1e7).toInt())
                        .targetSystem(targetSystem)
                        .targetComponent(targetComponent)
                        .missionType(MavMissionType.MAV_MISSION_TYPE_FENCE)
                        .build()
                    missionItems.add(missionItem)
                }
            }
        }
    }

    override fun writeToFile(kmz: KMZ): Completable {
        return Completable.fromCallable {
            val content: MutableMap<String, Any> = HashMap()
            content["hasReturnPoint"] = false
            content["hasReturnPointAltitude"] = false

            returnPoint?.let {
                content["hasReturnPoint"] = true
                content["returnPointLat"] = it.x
                content["returnPointLon"] = it.y

                if (!it.z.isNaN()) {
                    content["hasReturnPointAltitude"] = true
                    content["returnPointAlt"] = it.z
                }
            }

            content["items"] = fenceItems.map { it.exportAsObjectMap() }.toMutableList()

            writeFileUsingTemplate(
                "templates/geofence.template.xml",
                kmz.geofenceFilePath,
                content,
                "geofence",
                log
            )
        }
    }

    override val allowedCommands: List<MavCmd>
        get() = listOf(
            MavCmd.MAV_CMD_NAV_FENCE_RETURN_POINT,
            MavCmd.MAV_CMD_NAV_FENCE_CIRCLE_INCLUSION,
            MavCmd.MAV_CMD_NAV_FENCE_CIRCLE_EXCLUSION,
            MavCmd.MAV_CMD_NAV_FENCE_POLYGON_VERTEX_INCLUSION,
            MavCmd.MAV_CMD_NAV_FENCE_POLYGON_VERTEX_EXCLUSION
        )

    @Throws(MissionConverterException::class)
    private fun readFromFile(geofenceFilePath: String) {
        returnPoint = null
        fenceItems.clear()

        try {
            val factory = DocumentBuilderFactory.newInstance()
            val builder = factory.newDocumentBuilder()
            val doc = builder.parse(geofenceFilePath)
            doc.documentElement.normalize()

            // Extract return point
            val returnElement = doc.getElementsByTagName("return_point").item(0) as? Element
            returnElement?.let {
                returnPoint = stringToCoordinate(it.textContent)
            }

            // Get all geofence items
            val geofenceElements = doc.getElementsByTagName("geofenceItem")
            for (i in 0 until geofenceElements.length) {
                val geofenceItem = geofenceElements.item(i) as Element
                val type = getAttributeByName(geofenceItem, "type")
                val inclusion = getAttributeByName(geofenceItem, "inclusion")

                // Process circle type geofence
                if (type == "circle") {
                    val center = getElementByName(geofenceItem, "center")
                    val radius = getElementByName(geofenceItem, "radius").toDouble()
                    val isIncluded = inclusion == "true"

                    val fenceCircle = FenceCircle(stringToCoordinate(center), radius, isIncluded)
                    fenceItems.add(fenceCircle)
                }

                // Process polygon type geofence
                if (type == "polygon") {
                    val vertexElements = geofenceItem.getElementsByTagName("vertex")

                    val coordinateList: MutableList<Coordinate> = ArrayList()
                    for (j in 0 until vertexElements.length) {
                        val vertex = vertexElements.item(j).textContent
                        coordinateList.add(stringToCoordinate(vertex))
                    }

                    // Close the polygon by adding the first vertex at the end
                    if (coordinateList.isNotEmpty()) {
                        coordinateList.add(coordinateList[0])
                    }

                    val isIncluded = inclusion == "true"
                    val fencePolygon =
                        FencePolygon(coordinateList.toTypedArray<Coordinate>(), isIncluded)
                    fenceItems.add(fencePolygon)
                }
            }
        } catch (e: ParserConfigurationException) {
            throw MissionConverterException("Cannot parse " + geofenceFilePath + "! " + e.message)
        } catch (e: SAXException) {
            throw MissionConverterException("Cannot parse " + geofenceFilePath + "! " + e.message)
        } catch (e: IOException) {
            throw MissionConverterException("Cannot parse " + geofenceFilePath + "! " + e.message)
        }
    }

    fun getFenceItems(): List<FenceItem> {
        return fenceItems
    }

    abstract class FenceItem(inclusion: Boolean) {
        val isInclusion: Boolean = inclusion
        protected val geometryFactory: GeometryFactory = GeometryFactory()

        fun isAllowed(coordinate3D: LocationCoordinate3D): Boolean {
            return isAllowed(
                geometryFactory.createPoint(
                    Coordinate(
                        coordinate3D.latitude,
                        coordinate3D.longitude
                    )
                )
            )
        }

        abstract fun isAllowed(point: Point): Boolean

        abstract fun exportAsObjectMap(): Map<String, Any>
    }

    class FencePolygon(vertices: Array<Coordinate>?, inclusion: Boolean) : FenceItem(inclusion) {
        private val polygon: Polygon

        init {
            val geometryFactory = GeometryFactory()
            val ring = geometryFactory.createLinearRing(vertices)
            polygon = geometryFactory.createPolygon(ring, null)
        }

        override fun isAllowed(point: Point): Boolean {
            return isInclusion == polygon.contains(point)
        }

        override fun exportAsObjectMap(): Map<String, Any> {
            val objectMap: MutableMap<String, Any> = HashMap()
            objectMap["isPolygon"] = true
            objectMap["isCircle"] = false
            objectMap["type"] = "polygon"
            objectMap["inclusion"] = isInclusion

            val verticesList: MutableList<Map<String, Any>> = ArrayList()
            vertices.forEach {
                val vertexObject: MutableMap<String, Any> = HashMap()
                vertexObject["pointLat"] = it.x
                vertexObject["pointLon"] = it.y
                verticesList.add(vertexObject)
            }

            objectMap["vertices"] = verticesList
            return objectMap
        }

        val vertices: Array<Coordinate>
            get() = polygon.coordinates
    }

    class FenceCircle(center: Coordinate?, val radius: Double, inclusion: Boolean) :
        FenceItem(inclusion) {
        private val center: Point = geometryFactory.createPoint(center)

        override fun isAllowed(point: Point): Boolean {
            val distance =
                getDistanceInMeterFromTwoGPSLocations(point.x, point.y, center.x, center.y)
            return if (isInclusion) distance <= radius else distance > radius
        }

        override fun exportAsObjectMap(): Map<String, Any> {
            val objectMap: MutableMap<String, Any> = HashMap()
            objectMap["isPolygon"] = false
            objectMap["isCircle"] = true
            objectMap["type"] = "circle"
            objectMap["inclusion"] = isInclusion

            objectMap["radius"] = radius
            objectMap["centerLat"] = getCenter().x
            objectMap["centerLon"] = getCenter().y
            return objectMap
        }

        fun getCenter(): Coordinate {
            return center.coordinate
        }
    }

    companion object {
        fun fromKMZ(
            connectionWR: WeakReference<MavConnection>,
            missionManagerWR: WeakReference<MissionManager>,
            kmz: KMZ
        ): Single<GeofenceMission> {
            return kmz.extractKMZ()
                .andThen(Single.fromCallable {
                    val mission = GeofenceMission(connectionWR, missionManagerWR)
                    mission.readFromFile(kmz.geofenceFilePath)
                    mission
                })
        }

        @Throws(MissionConverterException::class)
        private fun stringToCoordinate(str: String): Coordinate {
            val parts = str.split(",".toRegex()).dropLastWhile { it.isEmpty() }.toTypedArray()

            if (parts.size < 2 || parts.size > 3) {
                throw MissionConverterException("Cannot parse '$str' to coordinates!")
            }

            val latitude = parts[0].trim { it <= ' ' }.toDouble()
            val longitude = parts[1].trim { it <= ' ' }.toDouble()

            if (parts.size == 3) {
                val altitude = parts[2].trim { it <= ' ' }.toDouble()
                return Coordinate(latitude, longitude, altitude)
            }

            return Coordinate(latitude, longitude)
        }

        @Throws(MissionConverterException::class)
        private fun getElementByName(element: Element, name: String): String {
            val nodeList = element.getElementsByTagName(name)

            if (nodeList.length != 1) {
                throw MissionConverterException("Cannot find '$name' element")
            }

            return nodeList.item(0).textContent.trim { it <= ' ' }
        }

        @Throws(MissionConverterException::class)
        private fun getAttributeByName(element: Element, name: String): String {
            val attribute = element.getAttribute(name)
                ?: throw MissionConverterException("Cannot find '$name' attribute")

            return attribute
        }
    }
}
