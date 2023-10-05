package com.example.map

import android.Manifest
import android.content.Context
import android.content.Intent
import android.content.SharedPreferences
import android.content.pm.PackageManager
import androidx.appcompat.app.AppCompatActivity
import android.os.Bundle
import android.util.Log
import android.view.View
import android.widget.Toast
import androidx.core.content.ContextCompat
import androidx.lifecycle.Observer
import androidx.lifecycle.ViewModelProvider
import androidx.recyclerview.widget.LinearLayoutManager
import androidx.recyclerview.widget.RecyclerView
import com.example.map.databinding.ActivityMainBinding
import com.google.android.gms.maps.CameraUpdateFactory
import com.google.android.gms.maps.GoogleMap
import com.google.android.gms.maps.OnMapReadyCallback
import com.google.android.gms.maps.SupportMapFragment
import com.google.android.gms.maps.model.LatLng
import com.google.android.gms.maps.model.MarkerOptions
import com.google.android.material.datepicker.CalendarConstraints
import com.google.android.material.datepicker.DateValidatorPointBackward
import com.google.android.material.datepicker.DateValidatorPointForward
import com.google.android.material.datepicker.MaterialDatePicker
import java.util.Calendar
import java.util.Date
import kotlinx.coroutines.*
import retrofit2.Call
import retrofit2.Callback
import retrofit2.Response
import retrofit2.http.Path
import java.text.SimpleDateFormat
import java.util.Locale

class MainActivity : AppCompatActivity(){

    private val binding: ActivityMainBinding by lazy{
        ActivityMainBinding.inflate(layoutInflater)
    }
    private lateinit var diaryAdapter: DiaryAdapter
    var diaryList =listOf<Diary>()

    override fun onCreate(savedInstanceState: Bundle?) {
        super.onCreate(savedInstanceState)
//        binding=ActivityMainBinding.inflate(layoutInflater)
        diaryAdapter = DiaryAdapter()
        setContentView(binding.root)

        print("AI다이어리 탭을 실행합니다")

        print("일기 데이터 적용을 시작합니다")
        binding.recyclerViewDiary.apply {
            adapter=diaryAdapter
            layoutManager=LinearLayoutManager(context)
            setHasFixedSize(true)

            print("어댑터 적용이 끝났습니다")
        }

        print("데이터를 불러옵니다")
        val currentDate = SimpleDateFormat("yyyyMMdd", Locale.getDefault()).format(Date())
        initList(currentDate)
        print("정상 갱신 완료")





        binding.planList.setOnClickListener(View.OnClickListener {
            // 이동하고자 하는 DiaryActivity를 시작하는 Intent를 생성
            val intent = Intent(this@MainActivity, PlanListActivity::class.java)

            // 필요한 경우, Intent에 데이터를 추가할 수 있음
            // intent.putExtra("key", value)

            // DiaryActivity로 이동
            startActivity(intent)

        })

        binding.aiDiary.setOnClickListener(View.OnClickListener {
            var intent=Intent(this@MainActivity,DiaryActivity::class.java )

            //ai diary페이지로 이동
            startActivity(intent)
        })

        binding.travelPlan.setOnClickListener(View.OnClickListener {
            var intent=Intent(this@MainActivity,MapActivity::class.java )

            //여행 플랜 만드는 페이지로 이동
            startActivity(intent)
        })

        binding.goMap.setOnClickListener(View.OnClickListener {
            var intent=Intent(this@MainActivity,MapActivity::class.java )

            //여행 플랜 만드는 페이지로 이동
            startActivity(intent)
        })




        //sharedPreference를 이용한 기기에 선택한 날짜 데이터 저장
        val sharedPreference = getSharedPreferences("CreateProfile", Context.MODE_PRIVATE)
        val editor: SharedPreferences.Editor = sharedPreference.edit()

        binding.calanderButtonCreate.setOnClickListener {
            val calendarConstraintBuilder = CalendarConstraints.Builder()
            calendarConstraintBuilder.setValidator(DateValidatorPointBackward.now())

            val builder = MaterialDatePicker.Builder.datePicker()
                .setTitleText("날짜를 선택하세요:)")
                .setSelection(MaterialDatePicker.todayInUtcMilliseconds())
                .setCalendarConstraints(calendarConstraintBuilder.build())

            val datePicker = builder.build()

            datePicker.addOnPositiveButtonClickListener {
                val calendar = Calendar.getInstance()
                calendar.time = Date(it)
                val calendarMilli = calendar.timeInMillis

                // 날짜 형식을 변환 (10/3/2023 -> 20231005)
                val year = calendar.get(Calendar.YEAR)
                val month = calendar.get(Calendar.MONTH) + 1 // 월은 0부터 시작하므로 1을 더해줌
                val day = calendar.get(Calendar.DAY_OF_MONTH)

                val formattedMonth = String.format("%02d", month)
                val formattedDay = String.format("%02d", day)
//20231004
                val settedDate = "$year$formattedMonth$formattedDay"

                binding.calanderButtonCreate.text = "$year/$month/$day"

                // SharedPreference
                editor.putLong("Die_Millis", calendarMilli)
                editor.apply()
                 Log.d("Die_Millis", "$calendarMilli") // Log 사용을 위해 추가해야 함

                // 선택한 날짜로 일기 조회
                Log.d("사용자가 설정한 날짜",settedDate)
                initList(settedDate)

            }
            datePicker.show(supportFragmentManager, datePicker.toString())
        }


    }

    private fun initList(selectedDate: String) {


//        val currentDate = SimpleDateFormat("yyyyMMdd", Locale.getDefault()).format(Date())
        val call = RetrofitClient.getRetrofitService.getDiaryByUserDay(selectedDate)


        call.enqueue(object: Callback<List<Diary>> {
            override fun onResponse(call: Call<List<Diary>>, response: Response<List<Diary>>) {
                Toast.makeText(applicationContext, "일기조회가 완료되었습니다", Toast.LENGTH_SHORT).show()
                if(response.isSuccessful) {
                    diaryList = response.body() ?: listOf()
                    diaryAdapter.setList(diaryList)
//                    Log.d("결과",diaryList.toString().substring(0,30))
                    diaryAdapter.notifyDataSetChanged()
                }
            }

            override fun onFailure(call: Call<List<Diary>>, t: Throwable) {
                Toast.makeText(applicationContext, "일기 조회에 실패했습니다", Toast.LENGTH_SHORT).show()
            }
        })
    }




}


