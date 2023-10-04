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

import com.google.android.gms.maps.model.Marker
import org.json.JSONException
import com.google.android.gms.maps.model.MarkerOptions
//import com.google.android.gms.maps.model.LatLng
import android.content.Intent
import com.google.android.gms.maps.model.BitmapDescriptorFactory
import android.graphics.BitmapFactory
import android.graphics.Bitmap
import android.os.Parcelable






class NaviActivity : AppCompatActivity(), OnMapReadyCallback {

    var latLng = LatLng(37.57996318, 126.887599016)
    var currentMarker: Marker? = null
    private var latLngList: ArrayList<LatLng>? = null


    private lateinit var googleMap: GoogleMap
    private val pointList = ArrayList<Double>()
    private lateinit var webSocketClient: WebSocketClient

    override fun onCreate(savedInstanceState: Bundle?) {
        super.onCreate(savedInstanceState)
        setContentView(R.layout.activity_navi)

        val bundle = intent.extras
        //val latLngList: ArrayList<LatLng>? = bundle?.getParcelableArrayList("pointsList")
        latLngList = intent.getParcelableArrayListExtra<Parcelable>("pointsList") as? ArrayList<LatLng>

        println("Received latLngList: $latLngList")


        val mapFragment = SupportMapFragment.newInstance()
        supportFragmentManager.beginTransaction()
            .replace(R.id.map, mapFragment)
            .commit()

        mapFragment.getMapAsync(this)
        /*
        val publishButton: Button = findViewById(R.id.publishButton)
        val resetButton: Button = findViewById(R.id.resetButton) // 리셋 버튼 추가
        val drivingButton: Button = findViewById(R.id.drivingButton)

        drivingButton.setOnClickListener {
            val intent = Intent(this@MainActivity, NaviActivity::class.java)
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
        */
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

                //val subscribeMsg = JSONObject()
                //subscribeMsg.put("op", "subscribe")
                //subscribeMsg.put("topic", "/road_point")
                val subscribeGPS = JSONObject()
                subscribeGPS.put("op", "subscribe")
                subscribeGPS.put("topic", "/gps")
                //this@NaviActivity.webSocketClient.send(subscribeMsg.toString())
                this@NaviActivity.webSocketClient.send(subscribeGPS.toString())
            }

            override fun onMessage(message: String?) {
                //println("get message")
                //println("Received message: $message")



                if (message != null && message.isNotBlank()) {

                    try {
                        val jsonObject = JSONObject(message)
                        val topic = jsonObject.getString("topic")
                        val msgObject = jsonObject.getJSONObject("msg")
                        //val dataString = msgObject.getString("data")

                        /*
                        if (jsonObject.getString("topic") == "/road_point") {
                            println("get road_point")
                            val dataString = jsonObject.getJSONObject("msg").getString("data")

                            val points = dataString.split(";")
                            println("$points")
                            // 경로를 그리기 위한 PolylineOptions 객체 생성
                            val polylineOptions = PolylineOptions()
                            for (point in points) {
                                val coordinates = point.trim().split(",")
                                if (coordinates.size == 2) {
                                    val easting = coordinates[0].trim().toDouble()
                                    val northing = coordinates[1].trim().toDouble()
                                    val latLng = convertUTMToLatLong(easting, northing, "52") // replace "33T" with your UTM zone if different
                                    //println("$latLng")
                                    polylineOptions.add(latLng)
                                    //println("add latlong to polyline")
                                }
                            }
                            println("$polylineOptions")
                            polylineOptions.color(ContextCompat.getColor(this@MainActivity, android.R.color.holo_red_dark))
                            polylineOptions.width(10f)
                            runOnUiThread {
                                println("line start")
                                googleMap.addPolyline(polylineOptions)
                                println("good")
                            }
                        }else if(topic == "/gps"){
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
                        if(topic == "/gps"){
                            //println("get gps")
                            //jsonObject.getJSONObject("msg").getString("data")
                            val latitude = jsonObject.getJSONObject("msg").getDouble("latitude")
                            val longitude = jsonObject.getJSONObject("msg").getDouble("longitude")

                            latLng = LatLng(latitude, longitude)  // 여기에서 LatLng 객체를 생성

                            // Add marker or update position on the map
                            runOnUiThread {
                                if (currentMarker == null) {
                                    // 커스텀 마커 이미지를 비트맵으로 변환
                                    val originalBitmap = BitmapFactory.decodeResource(resources, R.drawable.car)
                                    val width = (originalBitmap.width * 0.15).toInt()  // 여기에서 0.5는 50% 크기를 의미합니다.
                                    val height = (originalBitmap.height * 0.15).toInt()
                                    val smallMarker = Bitmap.createScaledBitmap(originalBitmap, width, height, false)

                                    // 비트맵을 BitmapDescriptor로 변환
                                    val customIcon = BitmapDescriptorFactory.fromBitmap(smallMarker)

                                    // 처음 마커를 생성
                                    currentMarker = googleMap.addMarker(
                                        MarkerOptions()
                                            .position(latLng)
                                            .title("GPS Location")
                                            .icon(customIcon)  // 크기가 조절된 아이콘 설정
                                    )
                                } else {
                                    // 마커 위치 업데이트
                                    currentMarker!!.setPosition(latLng)
                                }
                                googleMap.moveCamera(CameraUpdateFactory.newLatLng(latLng))
                            }
                            /*
                            runOnUiThread {
                                val gpsPosition = LatLng(latitude, longitude)
                                googleMap.addMarker(MarkerOptions().position(gpsPosition).title("GPS Location"))
                                googleMap.moveCamera(CameraUpdateFactory.newLatLng(gpsPosition))
                            }
                             */
                        }

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

    /*
    private fun publishData() {
        val dataJson = JSONObject()
        dataJson.put("op", "publish")
        dataJson.put("topic", "/point") //  /point라는 topic으로 메시지 publish

        val messageData = JSONObject()  // 메시지 형식을 {data: (string)}의 json형식으로 보낸다
        //messageData.put("data", pointList.joinToString(",")) // 위도와 경도를 쉼표로 구분한 문자열로 변환
        messageData.put("data", "37.582011495162206, 126.88994707387491,37.5809249336608, 126.88417050346895, 37.58008498101408, 126.88719808863462")

        dataJson.put("msg", messageData)

        webSocketClient.send(dataJson.toString())   // 메시지를 string으로 보냄
        println("success send massage")
    }
     */

    override fun onMapReady(gMap: GoogleMap) {

        googleMap = gMap
        //val latLng = LatLng(37.57996318, 126.887599016)
        googleMap.moveCamera(CameraUpdateFactory.newLatLngZoom(latLng, 18f))
        /*
        googleMap.setOnMapClickListener { latLng ->
            pointList.add(latLng.latitude)
            pointList.add(latLng.longitude)
            googleMap.addMarker(
                com.google.android.gms.maps.model.MarkerOptions()
                    .position(latLng)
                    .title("Marker")
            )
            println("pointList: $pointList")
        }

         */
        /*
        latLngList?.let { list ->

            if (list.size > 2) { // 리스트에 최소 3개의 좌표가 있는 경우만 처리
                val sublist = list.subList(2, list.size) // 앞의 두 좌표를 제외한 새로운 리스트 생성
                val polylineOptions = PolylineOptions()
                polylineOptions.addAll(sublist)
                polylineOptions.width(20f)
                polylineOptions.color(ContextCompat.getColor(this, android.R.color.holo_red_dark))
                googleMap.addPolyline(polylineOptions)
            }

        }
         */
        latLngList?.let { list ->
            val polylineOptions = PolylineOptions()
            polylineOptions.addAll(list)
            polylineOptions.width(20f)
            polylineOptions.color(ContextCompat.getColor(this, android.R.color.holo_red_dark))
            googleMap.addPolyline(polylineOptions)
        }

        initWebSocket()
    }
    /*
    fun convertUTMToLatLong(easting: Double, northing: Double, zone: String): LatLng {
        val crsFactory = CRSFactory()
        val ctFactory = CoordinateTransformFactory()

        val sourceCRS = crsFactory.createFromName("EPSG:326" + zone) // This assumes northern hemisphere. For southern hemisphere use "EPSG:327" + zone
        val targetCRS = crsFactory.createFromName("EPSG:4326") // WGS84
        val transform = ctFactory.createTransform(sourceCRS, targetCRS)

        val sourceCoordinate = ProjCoordinate(easting, northing)
        val targetCoordinate = ProjCoordinate()

        transform.transform(sourceCoordinate, targetCoordinate)

        return LatLng(targetCoordinate.y, targetCoordinate.x)
    }

     */
}

