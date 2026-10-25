# PC 仿真 —— 用宿主机 gcc 编译，不参与 Keil 工程
#
#   make        编译出 sim_ctrl
#   make run    跑一次（初始偏移 40mm）
#   make sweep  扫几个初始偏移，看大偏差下会不会振荡
#   make clean

CC      ?= gcc
CFLAGS  ?= -std=c99 -O2 -Wall -Wextra -I..
LDFLAGS ?= -lm
TARGET  := sim_ctrl
SRCS    := sim_main.c ../ball_ctrl_v2.c

.PHONY: all run sweep clean

all: $(TARGET)

$(TARGET): $(SRCS) ../ball_ctrl_v2.h
	$(CC) $(CFLAGS) -o $@ $(SRCS) $(LDFLAGS)

run: $(TARGET)
	./$(TARGET) 40

sweep: $(TARGET)
	@for x in 10 25 40 60; do \
		echo "--- 初始偏移 $$x mm ---"; \
		./$(TARGET) $$x | tail -7; \
	done

clean:
	rm -f $(TARGET) $(TARGET).exe
