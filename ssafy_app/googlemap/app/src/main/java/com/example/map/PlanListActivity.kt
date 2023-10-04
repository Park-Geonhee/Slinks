package com.example.map

import android.content.Intent
import android.os.Bundle
import android.view.View
import android.widget.GridView
import androidx.appcompat.app.AppCompatActivity
import com.example.map.databinding.ActivityMainBinding
import com.example.map.databinding.ActivityPlanListBinding

class PlanListActivity : AppCompatActivity() {


    private lateinit var binding: ActivityPlanListBinding
    override fun onCreate(savedInstanceState: Bundle?) {
        super.onCreate(savedInstanceState)
        binding=ActivityPlanListBinding.inflate(layoutInflater)
        setContentView(binding.root)



        binding.btnDetail.setOnClickListener({
            val intent = Intent(this@PlanListActivity, PlanDetailActivity::class.java)

            startActivity(intent)
        })


        binding.btnDiary.setOnClickListener({
            val intent = Intent(this@PlanListActivity, MainActivity::class.java)

            startActivity(intent)
        })

        binding.btnTrvelPlan.setOnClickListener({
            val intent=Intent(this@PlanListActivity, MapActivity::class.java)
            startActivity(intent)
        })
//        val gridView: GridView = findViewById(R.id.gridView)
//        val adapter = PlanListAdapter(this, getPlanItems())
//        gridView.adapter = adapter


    }
//
//    private fun getPlanItems(): List<PlanItem> {
//        // PlanItem 객체들을 생성하고 리스트로 반환
//
//        val planItems: MutableList<PlanItem> = mutableListOf()
//        planItems.add(PlanItem("1", "나의 작년이 담긴 갬성 코스", R.drawable.root1))
//        planItems.add(PlanItem("2", "가자! 상암으로~", R.drawable.root2))
//        planItems.add(PlanItem("3", "그다음 플래ㅐㅐㅐㄴ", R.drawable.root3))
//        planItems.add(PlanItem("4", "새로운 여행 코스~!!!", R.drawable.root4))
//        planItems.add(PlanItem("5", "너무 많다ㅏㅏㅏㅏ", R.drawable.root5))
//        planItems.add(PlanItem("6", "야호 여름이다!!", R.drawable.root6))
//
//        // 나머지 아이템들도 추가해주세요.
//        return planItems
//    }
}
