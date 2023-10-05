package com.example.map
import android.util.Log
import android.view.LayoutInflater
import android.view.ViewGroup
import androidx.recyclerview.widget.ListAdapter
import androidx.recyclerview.widget.RecyclerView
import com.bumptech.glide.Glide
import com.example.map.databinding.ItemRecycleDiaryBinding

class DiaryAdapter:RecyclerView.Adapter<DiaryAdapter.MyView>() {

        private var diaryList = listOf<Diary>()

        inner class MyView(private val binding: ItemRecycleDiaryBinding): RecyclerView.ViewHolder(binding.root) {
            fun bind(pos: Int) {

                binding.placeName.text = diaryList[pos].place?.name ?: "장소명"
                binding.placeType.text = diaryList[pos].place?.placeType?.value ?: "장소 타입"
                binding.placeAddress.text = diaryList[pos].place?.address ?: "주소"

                var time:String=diaryList[pos].createdDate?:"202310010000"

                var tt:String=time.substring(8,10)
                tt+=":"
                tt+=time.substring(10,12)

                binding.diaryRegTime.text=tt

                binding.diaryContent.text = diaryList[pos].content

                val imgNameee:String=diaryList[pos].file?.imgName?:""

                Glide.with(binding.root)
                    .load("http://j9a701.p.ssafy.io/uploads/" + imgNameee)
                    .centerCrop() // 이미지를 중앙으로 잘라서 표시합니다.
                    .into(binding.diaryImg) // ImageView에 이미지를 표시합니다.



                Log.d("진행상황", "굿굿")
            }
        }

        override fun onCreateViewHolder(parent: ViewGroup, viewType: Int): MyView {
            val view =ItemRecycleDiaryBinding.inflate(LayoutInflater.from(parent.context), parent, false)
            return MyView(view)
        }

        override fun onBindViewHolder(holder: MyView, position: Int) {
            holder.bind(position)
        }

        override fun getItemCount(): Int {
            return diaryList.size
        }

        fun setList(list: List<Diary>) {
            diaryList = list
//            Log.d("어댑터 진행상황", "일단 리스트 옮기기 까지 성공"+diaryList.toString().substring(0,30))
        }
    }