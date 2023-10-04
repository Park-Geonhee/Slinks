package com.example.map

import File

data class Plan(

    val id: Long,
    val title: String,
    val depDatetime: String?,
    val isPublic: Boolean,
    val user: User?,
    val file: File?
)
