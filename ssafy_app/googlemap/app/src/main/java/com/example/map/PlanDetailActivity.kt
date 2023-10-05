package com.example.map

import android.content.Intent
import androidx.appcompat.app.AppCompatActivity
import android.os.Bundle
import android.view.View
import android.widget.TimePicker
import com.example.map.databinding.ActivityPlanDetailBinding
import java.text.SimpleDateFormat
import java.util.Calendar
import java.util.Locale

import androidx.core.content.ContextCompat
import com.google.android.gms.maps.CameraUpdateFactory
import com.google.android.gms.maps.GoogleMap
import com.google.android.gms.maps.OnMapReadyCallback
import com.google.android.gms.maps.model.LatLng
import org.java_websocket.client.WebSocketClient

import com.google.android.gms.maps.model.PolylineOptions

import com.google.android.gms.maps.model.Marker


import android.os.Parcelable
import android.util.Log
import android.widget.Toast
import com.google.android.gms.maps.MapView
import android.widget.FrameLayout
import kotlinx.coroutines.CoroutineScope
import kotlinx.coroutines.Dispatchers
import kotlinx.coroutines.launch
import retrofit2.Call
import retrofit2.Callback
import retrofit2.Response


class PlanDetailActivity : AppCompatActivity() , TimePicker.OnTimeChangedListener, OnMapReadyCallback{

    var latLng = LatLng(37.57996318, 126.887599016)
    var currentMarker: Marker? = null
    private var latLngList: ArrayList<LatLng>? = null

     lateinit var place_name:ArrayList<String>


    private lateinit var googleMap: GoogleMap
    private val pointList = ArrayList<Double>()
    //val receivedLatLngList: ArrayList<LatLng>? = intent.getParcelableArrayListExtra("pointsList")


    private lateinit var webSocketClient: WebSocketClient
    //private lateinit var mapView: MapView
    private lateinit var mapView: MapView

    private lateinit var binding: ActivityPlanDetailBinding

    override fun onCreate(savedInstanceState: Bundle?) {
        place_name = ArrayList()
        super.onCreate(savedInstanceState)
        latLngList = intent.getParcelableArrayListExtra<Parcelable>("pointsList") as? ArrayList<LatLng>

        binding = ActivityPlanDetailBinding.inflate(layoutInflater)
        setContentView(binding.root)

        val recreatePathButton: FrameLayout = findViewById(R.id.btn_recreate_path)
        recreatePathButton.setOnClickListener {
            val intent = Intent(this@PlanDetailActivity, MapActivity::class.java)
            startActivity(intent)
            finish()  // 현재 액티비티를 종료 (필요하다면 추가하세요)
        }


        binding.startGuide.setOnClickListener {
            val intent = Intent(this@PlanDetailActivity, NaviActivity::class.java)
            latLngList?.let {
                intent.putParcelableArrayListExtra("pointsList", it)
            }
            startActivity(intent)
        }

        binding.btnPrevious.setOnClickListener(View.OnClickListener {


            val intent = Intent(this@PlanDetailActivity, PlanListActivity::class.java)

            // 필요한 경우, Intent에 데이터를 추가할 수 있음
            // intent.putExtra("key", value)

            // 이동
            startActivity(intent)

        })

        //장소들 받아오기
//        CoroutineScope(Dispatchers.Default).launch {
//            val call=RetrofitClient.getRetrofitService.getPlanPlacesByPlanId(1)
//
//
//            call.enqueue(object: Callback<List<PlanPlace>> {
//                override fun onResponse(call: Call<List<PlanPlace>>, response: Response<List<PlanPlace>>) {
//                    Toast.makeText(applicationContext, "장소리스트 받아오기 완료", Toast.LENGTH_SHORT).show()
//                    if(response.isSuccessful) {
//                        var placeList = response.body() ?: listOf()
//
//                        Log.d("장소들", placeList.toString())
//
//
//
//                        for (p in placeList) {
//                            val placename =p.place?.name?:"-"
//
//                            if (p != null && p.place != null && p.place.name != null) {
//                              place_name.add(placename)
//                            }
//                        }
//                        Log.d("추출된 장소 이름들", place_name.toString())
//
//                        val gridviewAdapter = GridviewAdapter(this@PlanDetailActivity, place_name)
//
//                        binding.gridView.adapter=gridviewAdapter
//
//
//
//
//                    }
//                }
//
//                override fun onFailure(call: Call<List<PlanPlace>>, t: Throwable) {
//                    Toast.makeText(applicationContext, "장소검생에 실패했습니다", Toast.LENGTH_SHORT).show()
//                }
//
//            })
//
//        }



        //현재 시간을 가져와 저장
        var cal = Calendar.getInstance()
        val hour = cal.get(Calendar.HOUR_OF_DAY)
        val minute = cal.get(Calendar.MINUTE)


        // 현재 시간을 가져와서 timesettingBtn에 표시
        updateButtonTime(hour, minute)
        /*
        binding.timesettingBtn.setOnClickListener {
            // TimePickerDialog를 표시하여 시간을 선택할 수 있도록 함
            val timePickerDialog = TimePickerDialog(
                this,
                TimePickerDialog.OnTimeSetListener { _: TimePicker, selectedHour: Int, selectedMinute: Int ->
                    // 사용자가 선택한 시간을 timesettingBtn에 반영
                    updateButtonTime(selectedHour, selectedMinute)
                },
                hour,
                minute,
                false
            )
            timePickerDialog.show()
        }

         */

        mapView = findViewById(R.id.map)
        mapView.onCreate(savedInstanceState)
        mapView.getMapAsync(this)

    }

    override fun onMapReady(gMap: GoogleMap) {

        googleMap = gMap
        val latLng = LatLng(37.57996318, 126.887599016) // 이건 기본 카메라 위치이니, 필요한 경우 다른 곳으로 바꿔도 됩니다.
        googleMap.moveCamera(CameraUpdateFactory.newLatLngZoom(latLng, 15f))

        // Intent에서 데이터 가져오기
        val receivedLatLngList: ArrayList<LatLng>? = intent.getParcelableArrayListExtra<Parcelable>("pointsList") as? ArrayList<LatLng>

        // 가져온 데이터로 Polyline 그리기
        receivedLatLngList?.let { list ->
            val polylineOptions = PolylineOptions()
            polylineOptions.addAll(list)
            polylineOptions.width(20f)
            polylineOptions.color(ContextCompat.getColor(this, android.R.color.holo_red_dark))
            googleMap.addPolyline(polylineOptions)
        }
    }


    override fun onTimeChanged(p0: TimePicker?, p1: Int, p2: Int) {
        TODO("Not yet implemented")
    }

    fun updateButtonTime(hour: Int, minute: Int) {
        // 선택한 시간을 timesettingBtn에 표시
        val timeFormat = SimpleDateFormat("a hh:mm", Locale.getDefault())
        val calendar = Calendar.getInstance()
        calendar.set(Calendar.HOUR_OF_DAY, hour)
        calendar.set(Calendar.MINUTE, minute)
        val formattedTime = timeFormat.format(calendar.time)
        //binding.timesettingBtn.text = formattedTime
    }


    override fun onResume() {
        super.onResume()
        mapView.onResume()
    }

    override fun onPause() {
        super.onPause()
        mapView.onPause()
    }

    override fun onDestroy() {
        super.onDestroy()
        mapView.onDestroy()
    }

    override fun onLowMemory() {
        super.onLowMemory()
        mapView.onLowMemory()
    }


}
