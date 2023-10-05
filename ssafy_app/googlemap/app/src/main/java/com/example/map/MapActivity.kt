package com.example.map

import File
import android.content.Intent
import android.content.res.Resources
import android.os.Bundle
import android.util.Log
import android.view.View
import android.widget.ImageView
import android.widget.LinearLayout
import android.widget.Toast
import androidx.appcompat.app.AppCompatActivity
import com.bumptech.glide.Glide
import com.bumptech.glide.request.RequestOptions
import com.example.map.databinding.ActivityMapBinding


import com.google.android.gms.maps.*
import com.google.android.gms.maps.model.LatLng
import com.google.android.gms.maps.model.Marker
import com.google.android.gms.maps.model.MarkerOptions
import kotlinx.coroutines.CoroutineScope
import kotlinx.coroutines.Dispatchers
import kotlinx.coroutines.launch
import retrofit2.Call
import retrofit2.Callback
import retrofit2.Response

class MapActivity : AppCompatActivity(), OnMapReadyCallback, GoogleMap.OnMarkerClickListener {

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
                    // Retrofit을 사용하여 API 호출
                    val response: PlanPlace = RetrofitClient.getRetrofitService.createPlanPlace(planplace)
                        Log.d("response값 확인 ",response.toString())
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




