package com.example.map

import File
import android.content.Intent
import android.content.res.Resources
import android.os.Bundle
import android.util.Log
import android.view.View
import android.widget.Button
import android.widget.ImageView
import android.widget.LinearLayout
import android.widget.Toast
import androidx.appcompat.app.AppCompatActivity
import androidx.core.content.ContextCompat
import com.bumptech.glide.Glide
import com.bumptech.glide.request.RequestOptions
import com.example.map.databinding.ActivityMapBinding


import com.google.android.gms.maps.*
import com.google.android.gms.maps.model.LatLng
import com.google.android.gms.maps.model.Marker
import com.google.android.gms.maps.model.MarkerOptions
import com.google.android.gms.maps.model.PolylineOptions
import kotlinx.coroutines.CoroutineScope
import kotlinx.coroutines.Dispatchers
import kotlinx.coroutines.launch
import org.java_websocket.client.WebSocketClient
import org.java_websocket.handshake.ServerHandshake
import org.json.JSONException
import org.json.JSONObject
import retrofit2.Call
import retrofit2.Callback
import retrofit2.Response
import java.net.URI
import java.net.URISyntaxException


import com.google.android.gms.maps.CameraUpdateFactory
import com.google.android.gms.maps.GoogleMap
import com.google.android.gms.maps.OnMapReadyCallback


import org.locationtech.proj4j.CRSFactory
import org.locationtech.proj4j.CoordinateTransformFactory
import org.locationtech.proj4j.ProjCoordinate


class MapActivity : AppCompatActivity(), OnMapReadyCallback, GoogleMap.OnMarkerClickListener {
    //private lateinit var googleMap: GoogleMap
    private lateinit var webSocketClient: WebSocketClient
    private lateinit var polylineOptions: PolylineOptions
    private val crsFactory = CRSFactory()
    private val ctFactory = CoordinateTransformFactory()

    private val sourceCRS = crsFactory.createFromName("EPSG:326" + 52) // This assumes northern hemisphere. For southern hemisphere use "EPSG:327" + zone
    private val targetCRS = crsFactory.createFromName("EPSG:4326") // WGS84
    private val transform = ctFactory.createTransform(sourceCRS, targetCRS)
    private val targetCoordinate = ProjCoordinate()
    companion object {
        const val TAG = "MapActivity"
    }

    lateinit var binding: ActivityMapBinding
    var placeXYList = listOf<LatLngEntity>()
    private lateinit var nextPlaces: List<Coordination>
    private val markerInfoMap = HashMap<Marker?, Place?>()
    private lateinit var mapView: MapView
    private lateinit var googleMap: GoogleMap
    private var currentMarker: Marker? = null
    lateinit var nowPlace: Place
    lateinit var searchResult:List<Place>
    private val pointList = ArrayList<LatLng>()

    override fun onCreate(savedInstanceState: Bundle?) {
        super.onCreate(savedInstanceState)

        binding = ActivityMapBinding.inflate(layoutInflater)
        setContentView(binding.root)

        this.mapView = binding.mapView
        mapView.onCreate(savedInstanceState)
        mapView.getMapAsync(this@MapActivity)

        binding.aiDiary.setOnClickListener(View.OnClickListener {
            // 이동하고자 하는 DiaryActivity를 시작하는 Intent를 생성
            val intent = Intent(this@MapActivity, MainActivity::class.java)

            // 필요한 경우, Intent에 데이터를 추가할 수 있음
            // intent.putExtra("key", value)

            // DiaryActivity로 이동
            startActivity(intent)

        })
        binding.planList.setOnClickListener(View.OnClickListener {
            // 이동하고자 하는 DiaryActivity를 시작하는 Intent를 생성
            val intent = Intent(this@MapActivity, PlanListActivity::class.java)

            // DiaryActivity로 이동
            startActivity(intent)
        })

        val detailViewButton: Button = findViewById(R.id.detailViewButton)
        detailViewButton.setOnClickListener {
            if (pointList.isEmpty()) {
                // 경로가 없을 때 경고 메시지 띄우기
                Toast.makeText(this@MapActivity, "경유지를 추가해서 계획을 만드세요.", Toast.LENGTH_SHORT).show()
            } else {
                // 경로가 있을 때 PlanDetailActivity로 이동
                val intent = Intent(this@MapActivity, PlanDetailActivity::class.java)
                intent.putParcelableArrayListExtra("pointsList", pointList)
                startActivity(intent)
            }
        }

        // binding.registerButton에 planPlace정보 저장
        binding.registerButton.setOnClickListener {
            // 이 부분에서 Plan 객체를 생성하고 id를 설정해야 합니다.
            val plan = Plan(
                id = 1,
                title = "상암동네 한 바퀴",
                depDatetime = "202310060012",
                isPublic = true,
                user = User(1,"유지나","wlskb@naver.com","ginajjang","서울 동대문구 망우로14가길 90"),
                file = File(6, "98d66ae0-d025-47de-983c-d94d91cb8b05.png","root1.png","/usr/local/lib/upload-dir/98d66ae0-d025-47de-983c-d94d91cb8b05.png")
            )

            // PlanPlace 객체를 생성할 때 plan 속성에 위에서 생성한 Plan 객체를 할당해야 합니다.
            val planplace: PlanPlace = PlanPlace(
//                id = 1, // PlanPlace의 id
                id = null, // PlanPlace의 id
                plan = plan, // Plan 객체를 할당
                place = nowPlace
            )
            Log.d("현재 로그","현재선택된 장소는 "+nowPlace.toString())


            // 경유지 추가 post 요청
            CoroutineScope(Dispatchers.IO).launch {
                try {
                    // Retrofit을 사용하여 API 호출k
                    val response: PlanPlace =
                        RetrofitClient.getRetrofitService.createPlanPlace(planplace)
                    Log.d("response값 확인 ",response.toString()?:"")
                    // 서버 응답을 처리할 수 있는 코드 작성
                    if (response.id != null) {
                        Log.d("PlanPlace 등록 성공", "PlanPlace를 성공적으로 등록했습니다.")

                        runOnUiThread {
                            Toast.makeText(applicationContext, "경유지가 추가되었습니다", Toast.LENGTH_SHORT).show()
                        }
                    } else {
                        Log.e("PlanPlace 등록 실패", "PlanPlace 등록에 실패했습니다.")
                        // Handle failure scenario if needed
                    }
                } catch (e: Exception) {
                    // 네트워크 오류 또는 예외 발생 시의 처리
                    Log.e("PlanPlace 등록 실패", "PlanPlace 등록 중 오류가 발생했습니다: ${e.message}")
                }
            }

        }

        val shareButton: Button = findViewById(R.id.shareButton)    //경로 생성하기
        shareButton.setOnClickListener {
            val planId: Long = 1L // 여기서 'L' 접미사를 사용하여 Long 타입으로 변환
            getPlanPlacesByPlanId(planId)
            println("click")

        }

        //장소검색
        binding.searchButton.setOnClickListener {
            val searchText = binding.placeSearch.text.toString()

            // TODO: searchText를 사용하여 장소를 검색하고 지도에 표시하는 작업 수행
            // 이 부분에 실제로 지도에 장소를 표시하는 코드를 작성해야 합니다.
//            val call=RetrofitClient.getRetrofitService.searchPlaceByKeyword(searchText)

            CoroutineScope(Dispatchers.Default).launch {
                val call=RetrofitClient.getRetrofitService.searchPlaceByKeyword(searchText)


                call.enqueue(object: Callback<List<Place>> {
                    override fun onResponse(call: Call<List<Place>>, response: Response<List<Place>>) {
                        Toast.makeText(applicationContext, "장소검색 완료", Toast.LENGTH_SHORT).show()
                        if(response.isSuccessful) {
                            searchResult = response.body() ?: listOf()


                            //검색결과를 지도화면에 나타내기
                            for (p in searchResult) {
                                val place =p
                                if (place != null && place.x != null && place.y != null) {
                                    val location = LatLng(place.x.toDouble(), place.y.toDouble())

                                    val markerOptions = MarkerOptions().apply {
                                        position(location)
                                        title(place.name)
                                        snippet(place.address)
                                    }

                                    // 마커 추가
                                    val marker = googleMap.addMarker(markerOptions)
                                    Log.d("지금 핀 꽃은 장소", marker.toString())
                                    marker?.showInfoWindow()

                                    // 마커와 장소 정보를 맵에 저장
                                    if (marker != null) {
                                        markerInfoMap[marker] = place
                                    }


                                }
                            }





                        }
                    }

                    override fun onFailure(call: Call<List<Place>>, t: Throwable) {
                        Toast.makeText(applicationContext, "장소검생에 실패했습니다", Toast.LENGTH_SHORT).show()
                    }
                })

            }



            // 예시로 토스트 메시지를 표시하는 코드
            // Toast.makeText(this, "검색어: $searchText", Toast.LENGTH_SHORT).show()
        }

    return Unit
    }
    fun getPlanPlacesByPlanId(planId: Long) {
        val call = RetrofitClient.getRetrofitService.GettingPlaces(planId)
        call.enqueue(object : Callback<Map<String, List<List<Double>>>> {
            override fun onResponse(call: Call<Map<String, List<List<Double>>>>, response: Response<Map<String, List<List<Double>>>>) {
                val rawCoordinationList = response.body()?.get("coordination")
                val pointsList = ArrayList<LatLng>()

                rawCoordinationList?.forEach { rawCoordination ->
                    if (rawCoordination.size >= 2) {
                        pointsList.add(LatLng(rawCoordination[0], rawCoordination[1]))
                    }
                }

                println("Parsed Coordination List: $pointsList")
                publishData(pointsList)
            }

            override fun onFailure(call: Call<Map<String, List<List<Double>>>>, t: Throwable) {
                println("API Request failed: ${t.message}")
            }
        })
    }

    /*websocket*/
    private fun initWebSocket() {
        val uri: URI
        try {
            uri = URI("ws://172.20.10.2:9090")
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
                this@MapActivity.webSocketClient.send(subscribeMsg.toString())
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


                        if (jsonObject.getString("topic") == "/road_point") {
                            println("get road_point nono")
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


                                    polylineOptions.add(latLng)
                                    pointList.add(latLng)
                                    // count += 1


                                    //println("add latlong to polyline")
                                }
                            }
                            println("Received latLngList: $pointList")
                            println("$polylineOptions")
                            polylineOptions.color(ContextCompat.getColor(this@MapActivity, android.R.color.holo_red_dark))
                            polylineOptions.width(10f)
                            runOnUiThread {
                                println("line start")
                                googleMap.addPolyline(polylineOptions)
                                println("good")
                            }
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

    private fun publishData(pointsList: List<LatLng>) {
        val dataJson = JSONObject()
        dataJson.put("op", "publish")
        dataJson.put("topic", "/point")

        val messageData = JSONObject()
        val dataString = pointsList.joinToString(",") { "${it.latitude},${it.longitude}" }
        messageData.put("data", dataString)

        dataJson.put("msg", messageData)

        webSocketClient.send(dataJson.toString())
        println("success send massage")
    }


    /**
     * onMapReady()
     * Map 이 사용할 준비가 되었을 때 호출
     * @param googleMap
     */
    override fun onMapReady(googleMap: GoogleMap) {
        this.googleMap = googleMap

        // DMC
        val p: Place = Place(
            7,
            "DMC",
            "서울특별시 마포구 중동 390",
            "02-305-3111",
            "37.577689",
            "126.892129",
            PlaceType.AG2
        )
        setupMarker(LatLngEntity(37.577689, 126.892129), p)
        currentMarker?.showInfoWindow()


        // 일기가 등록되었던 장소들 추출
        CoroutineScope(Dispatchers.Default).launch {
            val call = RetrofitClient.getRetrofitService.getDiaryByuserId(1)
            call.enqueue(object : Callback<List<Diary>> {
                override fun onResponse(call: Call<List<Diary>>, response: Response<List<Diary>>) {
//                Toast.makeText(applicationContext, "작성된 일기들을 모두 조회했습니다", Toast.LENGTH_SHORT).show()

                    Log.d("일기 조회결과", "작성된 일기들을 모두 조회했습니다")

                    if (response.isSuccessful) {
                        val diaryList = response.body() ?: listOf()
                        Log.d("일기 먼저 조회", diaryList.toString())

                        // 추출한 장소들 pin 등록
                        var lastMarker: Marker? = null
                        for (diary in diaryList) {
                            val place = diary.place
                            if (place != null && place.x != null && place.y != null) {
                                val location = LatLng(place.x.toDouble(), place.y.toDouble())

                                val markerOptions = MarkerOptions().apply {
                                    position(location)
                                    title(place.name)
                                    snippet(place.address)
                                }

                                // 마커 추가
                                val marker = googleMap.addMarker(markerOptions)
                                Log.d("지금 핀 꽃은 장소", marker.toString())
                                marker?.showInfoWindow()

                                // 마커와 장소 정보를 맵에 저장
                                if (marker != null) {
                                    markerInfoMap[marker] = place
                                }

                                //마지막 장소 저장
                                lastMarker = marker
                            }
                        }

                        // 마지막 장소로 초점
                        lastMarker?.showInfoWindow()
                        googleMap.setOnMarkerClickListener(this@MapActivity)

                        Log.d("장소등록결과", "장소들을 모두 pin으로 등록했습니다")
                    }

                    Log.d("pin등록", "완료")
                }

                override fun onFailure(call: Call<List<Diary>>, t: Throwable) {
                    Toast.makeText(applicationContext, "조회에 실패했습니다", Toast.LENGTH_SHORT).show()
                }
            })

        }
        googleMap.setOnMarkerClickListener(this)

        initWebSocket()

    }
    fun convertUTMToLatLong(easting: Double, northing: Double, zone: String): LatLng {

        val sourceCoordinate = ProjCoordinate(easting, northing)

        transform.transform(sourceCoordinate, targetCoordinate)

        return LatLng(targetCoordinate.y, targetCoordinate.x)
    }

    /**
     * setupMarker()
     * 선택한 위치의 marker 표시
     * @param locationLatLngEntity
     * @return
     */
    private fun setupMarker(locationLatLngEntity: LatLngEntity, placeInfo: Place) {
        val positionLatLng =
            LatLng(locationLatLngEntity.latitude!!, locationLatLngEntity.longitude!!)
        val markerOption = MarkerOptions().apply {
            position(positionLatLng)
            title(placeInfo.name)
            snippet(placeInfo.address)
        }

        val marker = googleMap.addMarker(markerOption)

        googleMap.mapType = GoogleMap.MAP_TYPE_NORMAL
        googleMap.moveCamera(CameraUpdateFactory.newLatLngZoom(positionLatLng, 15f))
        googleMap.animateCamera(CameraUpdateFactory.zoomTo(15f))

        markerInfoMap[marker] = placeInfo
    }

    override fun onStart() {
        super.onStart()
        mapView.onStart()
    }

    override fun onStop() {
        super.onStop()
        mapView.onStop()
    }

    override fun onResume() {
        super.onResume()
        mapView.onResume()
    }

    override fun onPause() {
        super.onPause()
        mapView.onPause()
    }

    override fun onLowMemory() {
        super.onLowMemory()
        mapView.onLowMemory()
    }

    override fun onDestroy() {
        mapView.onDestroy()
        super.onDestroy()
    }
    fun Resources.dpToPx(dp: Int): Int {
        return (dp * this.displayMetrics.density).toInt()
    }


    fun loadDiaryImages(images: List<String>?) {
        val linearLayout = findViewById<LinearLayout>(R.id.diaryImageContainer)
        linearLayout.removeAllViews()

        if (images.isNullOrEmpty()) {
            // 이미지가 없는 경우 처리할 로직을 여기에 추가하세요.
            // 예를 들어, 기본 이미지를 표시하거나 특정 메시지를 보여줄 수 있습니다.
        } else {
            val requestOptions = RequestOptions().centerCrop()

            Log.d("이미지 url들", images.toString())
            for (imageUrl in images) {
                val imageView = ImageView(this)
                imageView.layoutParams = LinearLayout.LayoutParams(
                    resources.dpToPx(100),
                    resources.dpToPx(100)
                )


                Glide.with(this)
                    .load("http://j9a701.p.ssafy.io/uploads/"+imageUrl)
                    .apply(requestOptions)
                    .into(imageView)

                linearLayout.addView(imageView)
            }
        }
    }



    /**
     * LatLngEntity data class
     *
     * @property latitude 위도 (ex. 37.5562)
     * @property longitude 경도 (ex. 126.9724)
     */
    data class LatLngEntity(
        var latitude: Double?,
        var longitude: Double?
    )

    override fun onMarkerClick(p0: Marker): Boolean {
        p0?.let {
            val placeInfo = markerInfoMap[it]
            placeInfo?.let {
                binding.locationName.text = placeInfo.name ?: "알수없음"
                binding.loactionType.text = placeInfo.placeType.value ?: "기타"
                binding.locationAddress.text = "주소    " + placeInfo.address ?: "알수없음"
                binding.locationPhone.text = "전화번호    " + placeInfo.phone ?: "-"
                nowPlace = placeInfo

            }

            //마이다이어리 불러오기
            CoroutineScope(Dispatchers.Default).launch {
                val call=RetrofitClient.getRetrofitService.getDiariesByUserAndPlace(1,nowPlace.id)

                call.enqueue(object : Callback<List<Diary>> {
                    override fun onResponse(call: Call<List<Diary>>, response: Response<List<Diary>>) {
//                Toast.makeText(applicationContext, "작성된 일기들을 모두 조회했습니다", Toast.LENGTH_SHORT).show()

                        Log.d("일기 조회결과", "이 장소의 일기들을 조회했습니다")

                        if (response.isSuccessful) {
                            val diaryList = response.body() ?: listOf()

                            val imageUrls = mutableListOf<String>()

// 일기 이미지 URL 추출
                            for (diary in diaryList) {
                                val imgUrl = diary.file?.imgName
                                if (imgUrl != null && imgUrl.isNotBlank()) {
                                    imageUrls.add(imgUrl)
                                }
                            }

// 추출한 이미지 URL 리스트를 사용하여 이미지 로드 및 표시
                            loadDiaryImages(imageUrls)



                            googleMap.setOnMarkerClickListener(this@MapActivity)

                            Log.d("이 장소의 일기 조회 결과", "일기 이미지들을 모두 띄웠습니다")
                        }


                    }

                    override fun onFailure(call: Call<List<Diary>>, t: Throwable) {
                        Toast.makeText(applicationContext, "조회에 실패했습니다", Toast.LENGTH_SHORT).show()
                    }
                })
            }


            return true
        }
        return false
    }
}




