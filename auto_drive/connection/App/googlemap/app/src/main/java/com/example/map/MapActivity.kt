package com.example.map

import android.Manifest
import android.content.pm.PackageManager
import androidx.appcompat.app.AppCompatActivity
import android.os.Bundle
import androidx.core.content.ContextCompat
import com.google.android.gms.maps.CameraUpdateFactory
import com.google.android.gms.maps.GoogleMap
import com.google.android.gms.maps.OnMapReadyCallback
import com.google.android.gms.maps.SupportMapFragment
import com.google.android.gms.maps.model.LatLng
import com.google.android.gms.maps.model.MarkerOptions

class MapActivity : AppCompatActivity(), OnMapReadyCallback {
    private lateinit var googleMap: GoogleMap

    override fun onCreate(savedInstanceState: Bundle?) {
        super.onCreate(savedInstanceState)
        setContentView(R.layout.activity_main)

        val mapFragment = SupportMapFragment.newInstance()
        supportFragmentManager.beginTransaction()
            .replace(R.id.map, mapFragment)
            .commit()

        mapFragment.getMapAsync(this)
    }



    override fun onMapReady(gMap: GoogleMap) {
        googleMap = gMap

        // 시작지점 설정
        val latLng = LatLng(37.589723, 127.057866)

        // 카메라 이동 및 확대
        val cameraUpdate = CameraUpdateFactory.newLatLngZoom(latLng, 18.0f)
        googleMap.animateCamera(cameraUpdate)

        // 위치 마커 추가
        val markerOptions = MarkerOptions().position(latLng).title("회기역")
        googleMap.addMarker(markerOptions)

        // 현위치 버튼 활성화
        // 위치 권한 확인
        if (ContextCompat.checkSelfPermission(
                this,
                Manifest.permission.ACCESS_FINE_LOCATION
            ) == PackageManager.PERMISSION_GRANTED
        ) {
            googleMap.isMyLocationEnabled = true
        } else {
//            //권한 없으면 일단 그냥 종료하는 걸로
//            print("권한이 없네요")
//           finish()
        }
    }

}
