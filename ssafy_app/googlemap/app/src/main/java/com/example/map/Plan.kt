package com.example.map

import File

data class Plan(
    val id: Long,
    val title: String?,
    val depDatetime: String?,
    val isPublic: Boolean = true,  // 기본값으로 true를 설정
    val user: User?,
    val file: File?
)
