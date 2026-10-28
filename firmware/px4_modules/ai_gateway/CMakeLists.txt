# PX4 v1.14+ 模块注册：把本目录路径加入 boards/.../cmake 的 px4_add_module 搜索，
# 或直接放入 src/modules/ai_gateway/ 后在 <target>.px4board 中加:
#   CONFIG_MODULES_AI_GATEWAY=y
px4_add_module(
	MODULE modules__ai_gateway
	MAIN ai_gateway
	SRCS
		ai_gateway.c
	DEPENDS
		modules__navigator
	)
