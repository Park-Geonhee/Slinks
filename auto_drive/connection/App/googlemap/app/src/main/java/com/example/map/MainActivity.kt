package com.example.map

import android.Manifest
import android.content.pm.PackageManager
import androidx.appcompat.app.AppCompatActivity
import android.os.Bundle
import android.widget.Button
import androidx.core.content.ContextCompat
import com.google.android.gms.maps.CameraUpdateFactory
import com.google.android.gms.maps.GoogleMap
import com.google.android.gms.maps.OnMapReadyCallback
import com.google.android.gms.maps.SupportMapFragment
import com.google.android.gms.maps.model.LatLng
import org.java_websocket.client.WebSocketClient
import org.java_websocket.handshake.ServerHandshake
import org.json.JSONObject
import java.net.URI
import java.net.URISyntaxException

import org.locationtech.proj4j.CRSFactory
import org.locationtech.proj4j.CoordinateTransformFactory
import org.locationtech.proj4j.ProjCoordinate
import com.google.android.gms.maps.model.PolylineOptions

import org.json.JSONException
import com.google.android.gms.maps.model.MarkerOptions
//import com.google.android.gms.maps.model.LatLng
import android.content.Intent





class MainActivity : AppCompatActivity(), OnMapReadyCallback {
    private lateinit var googleMap: GoogleMap
    private val pointList = ArrayList<LatLng>()
    private lateinit var webSocketClient: WebSocketClient
    //private lateinit var messageEditText: EditText // 메시지 입력 창을 위한 변수 추가

    private lateinit var polylineOptions: PolylineOptions
    private val crsFactory = CRSFactory()
    private val ctFactory = CoordinateTransformFactory()

    private val sourceCRS = crsFactory.createFromName("EPSG:326" + 52) // This assumes northern hemisphere. For southern hemisphere use "EPSG:327" + zone
    private val targetCRS = crsFactory.createFromName("EPSG:4326") // WGS84
    private val transform = ctFactory.createTransform(sourceCRS, targetCRS)
    private val targetCoordinate = ProjCoordinate()


    override fun onCreate(savedInstanceState: Bundle?) {
        super.onCreate(savedInstanceState)
        setContentView(R.layout.activity_main)

        val mapFragment = SupportMapFragment.newInstance()
        supportFragmentManager.beginTransaction()
            .replace(R.id.map, mapFragment)
            .commit()

        mapFragment.getMapAsync(this)

        //WebSocket 초기화
        //initWebSocket()

        val publishButton: Button = findViewById(R.id.publishButton)
        val resetButton: Button = findViewById(R.id.resetButton) // 리셋 버튼 추가
        val drivingButton: Button = findViewById(R.id.drivingButton)

        drivingButton.setOnClickListener {
            val intent = Intent(this@MainActivity, NaviActivity::class.java)

            // PolylineOptions 객체에서 LatLng 목록 가져오기

            //val pointsList = polylineOptions.points
            //val latLngList = ArrayList<LatLng>(pointsList)
            //println("Sended pointsList: $pointsList")
            intent.putParcelableArrayListExtra("pointsList", pointList)

            // Intent에 LatLng 목록 추가하기
            //intent.putParcelableArrayListExtra("pointsList", ArrayList(pointsList))
            startActivity(intent)
        }

        publishButton.setOnClickListener {
            publishData()
            println("publish")

        }
        resetButton.setOnClickListener {
            // 리셋 버튼 클릭 시 데이터 초기화
            pointList.clear()
            googleMap.clear() // 지도에서 마커 제거
        }

    }

    private fun initWebSocket() {
        val uri: URI
        try {
            uri = URI("ws://13.125.75.163:9090")
        } catch (e: URISyntaxException) {
            e.printStackTrace()
            return
        }

        webSocketClient = object : WebSocketClient(uri) {
            override fun onOpen(handshakeData: ServerHandshake?) {
                println("Connected to the server.")

                val subscribeMsg = JSONObject()
                subscribeMsg.put("op", "subscribe")
                subscribeMsg.put("topic", "/road_point")
                //val subscribeGPS = JSONObject()
                //subscribeGPS.put("op", "subscribe")
                //subscribeGPS.put("topic", "/gps")
                this@MainActivity.webSocketClient.send(subscribeMsg.toString())
                //this@MainActivity.webSocketClient.send(subscribeGPS.toString())
            }

            override fun onMessage(message: String?) {
                println("get message")
                println("Received message: $message")

                if (message != null && message.isNotBlank()) {
                    try {
                        val jsonObject = JSONObject(message)
                        val topic = jsonObject.getString("topic")
                        val msgObject = jsonObject.getJSONObject("msg")
                        //val dataString = msgObject.getString("data")


                        if (jsonObject.getString("topic") == "/road_point") {
                            println("get road_point")
                            val dataString = jsonObject.getJSONObject("msg").getString("data")

                            val points = dataString.split(";")
                            println("$points")
                            // 경로를 그리기 위한 PolylineOptions 객체 생성
                            val polylineOptions = PolylineOptions()

                            val skippedPoints = 2
                            var count = 0

                            for (point in points) {
                                val coordinates = point.trim().split(",")
                                if (coordinates.size == 2) {
                                    val easting = coordinates[0].trim().toDouble()
                                    val northing = coordinates[1].trim().toDouble()
                                    val latLng = convertUTMToLatLong(easting, northing, "52") // replace "33T" with your UTM zone if different
                                    //println("$latLng")
                                    //if (count >= skippedPoints) {
                                        //polylineOptions.add(latLng)
                                    //}

                                    polylineOptions.add(latLng)
                                    pointList.add(latLng)
                                   // count += 1


                                    //println("add latlong to polyline")
                                }
                            }
                            println("Received latLngList: $pointList")
                            println("$polylineOptions")
                            polylineOptions.color(ContextCompat.getColor(this@MainActivity, android.R.color.holo_red_dark))
                            polylineOptions.width(10f)
                            runOnUiThread {
                                println("line start")
                                googleMap.addPolyline(polylineOptions)
                                println("good")
                            }
                        }
                        /*
                        else if(topic == "/gps"){
                            println("get gps")
                            //jsonObject.getJSONObject("msg").getString("data")
                            val latitude = jsonObject.getJSONObject("msg").getDouble("latitude")
                            val longitude = jsonObject.getJSONObject("msg").getDouble("longitude")

                            // Add marker or update position on the map
                            runOnUiThread {
                                val gpsPosition = LatLng(latitude, longitude)
                                googleMap.addMarker(MarkerOptions().position(gpsPosition).title("GPS Location"))
                                googleMap.moveCamera(CameraUpdateFactory.newLatLng(gpsPosition))
                            }
                        }

                         */

                    } catch (e: JSONException) {
                        println("JSON parsing error: ${e.message}")
                    }catch (e: Exception) {
                        println("Error parsing message: ${e.message}")
                    }
                }
            }

            override fun onClose(code: Int, reason: String?, remote: Boolean) {
                println("Connection closed.")
            }

            override fun onError(e: Exception?) {
                println("An error occurred: ${e?.message}")
            }
        }

        webSocketClient.connect()
    }

    private fun publishData() {
        val dataJson = JSONObject()
        dataJson.put("op", "publish")
        dataJson.put("topic", "/point") //  /point라는 topic으로 메시지 publish

        val messageData = JSONObject()  // 메시지 형식을 {data: (string)}의 json형식으로 보낸다
        //messageData.put("data", pointList.joinToString(",")) // 위도와 경도를 쉼표로 구분한 문자열로 변환
        //messageData.put("data", pointList.joinToString(",") { "${it.latitude},${it.longitude}" })

        messageData.put("data", "37.582011495162206, 126.88994707387491,37.5809249336608, 126.88417050346895, 37.579406885912356, 126.88830023397516")

        dataJson.put("msg", messageData)

        webSocketClient.send(dataJson.toString())   // 메시지를 string으로 보냄
        println("success send massage")
    }


    override fun onMapReady(gMap: GoogleMap) {
        googleMap = gMap
        val latLng = LatLng(37.57996318, 126.887599016)
        googleMap.moveCamera(CameraUpdateFactory.newLatLngZoom(latLng, 15f))
        googleMap.setOnMapClickListener { latLng ->
            //pointList.add(latLng.latitude)
            //pointList.add(latLng.longitude)
            pointList.add(latLng)
            googleMap.addMarker(
                com.google.android.gms.maps.model.MarkerOptions()
                    .position(latLng)
                    .title("Marker")
            )
            println("pointList: $pointList")
        }
        initWebSocket()
    }

    fun convertUTMToLatLong(easting: Double, northing: Double, zone: String): LatLng {
        //val crsFactory = CRSFactory()
        // val ctFactory = CoordinateTransformFactory()

        //val sourceCRS = crsFactory.createFromName("EPSG:326" + zone) // This assumes northern hemisphere. For southern hemisphere use "EPSG:327" + zone
        //val targetCRS = crsFactory.createFromName("EPSG:4326") // WGS84
        //val transform = ctFactory.createTransform(sourceCRS, targetCRS)

        val sourceCoordinate = ProjCoordinate(easting, northing)
        // val targetCoordinate = ProjCoordinate()

        transform.transform(sourceCoordinate, targetCoordinate)

        return LatLng(targetCoordinate.y, targetCoordinate.x)
    }
}